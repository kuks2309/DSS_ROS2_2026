#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>   // JPEG 디코딩용
#include <nats/nats.h>
#include "dss.pb.h"
#include "defaultGateway.h"
#include "nlohmann/json.hpp"
using json = nlohmann::json;
#define MAX_SUBS (64)   // 동시에 최대 64개 구독 보유

// ==================== NATS 클라이언트 보관 ====================
struct NatsClient {
    natsConnection*      conn = nullptr;
    natsSubscription*    subs[MAX_SUBS]{};
    int                  count = 0;
};

class DSSToROSImageNode : public rclcpp::Node {
public:
    using TopicHandler     = std::function<void(const std::string& subject, const char* data, int len)>;
    struct TopicCtx     { TopicHandler*     fn; };

    // NATS
    NatsClient nats_;
    std::vector<std::unique_ptr<TopicHandler>>            topicHandlers_;
    std::vector<std::unique_ptr<TopicCtx>>                rawCtx_;
    rclcpp::TimerBase::SharedPtr                          timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

    sensor_msgs::msg::Image createImage( dss::DSSImage& nats_msg) {
         double stamp_sec = nats_msg.header().stamp();
        
        const uint8_t* jpeg_data = (const uint8_t*)nats_msg.data().data();
        size_t jpeg_size = nats_msg.data().size();
        
        sensor_msgs::msg::Image msg;

        // JPEG → OpenCV 이미지로 디코드 (BGR)
        std::vector<uint8_t> jpeg_buffer(jpeg_data, jpeg_data + jpeg_size);
        cv::Mat bgr = cv::imdecode(jpeg_buffer, cv::IMREAD_COLOR);
        if (bgr.empty()) {
            throw std::runtime_error("JPEG decode failed!");
        }

        // BGR → RGB 변환
        cv::Mat rgb;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);

        // ROS 메시지에 채우기
        rclcpp::Time ros_stamp(
                static_cast<int64_t>(stamp_sec * 1e9),  // nanoseconds
                RCL_ROS_TIME
        );        
        msg.header.stamp    = ros_stamp;
        msg.header.frame_id = "camera";

        //RCLCPP_INFO(rclcpp::get_logger("image_bridge"),"Image stamp = %ld.%09u",msg.header.stamp.sec,msg.header.stamp.nanosec);        

        msg.height = rgb.rows;
        msg.width  = rgb.cols;
        msg.encoding = "rgb8";
        msg.is_bigendian = false;
        msg.step = rgb.cols * 3;

        msg.data.resize(rgb.rows * rgb.cols * 3);
        std::memcpy(msg.data.data(), rgb.data, msg.data.size());

        return msg;
    }    

public:
    DSSToROSImageNode() : Node("DSSToROSImageNode") {
        std::string kNatsUrl = "nats://" + getDefaultGateway()+ ":4222";
        RCLCPP_INFO(get_logger(), kNatsUrl.c_str());
        natsStatus s = natsConnection_ConnectTo(&nats_.conn, kNatsUrl.c_str());
        if (s != NATS_OK) {
            std::cerr << "NATS connect failed: " << natsStatus_GetText(s) << std::endl;
            return;
        }     
        
        timer_   = this->create_wall_timer(std::chrono::seconds(3), std::bind(&DSSToROSImageNode::onTick, this));
        subscribeTopicRaw("dss.sensor.camera.rgb",
            [this](const std::string& subject, const char* bytes, int len)
            {
                dss::DSSImage img_msg;
                if (!img_msg.ParseFromArray(bytes, len)) {
                    std::cerr << "Failed to parse DSSImage protobuf message\n";
                    return; 
                }
                pub_->publish(createImage(img_msg));
            }
        );
        
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("/dss/sensor/camera/rgb",10);
        RCLCPP_INFO(get_logger(), "[NATS]dss.sensor.camera.rgb → [ROS2]/dss/sensor/camera/rgb");
    }

    ~DSSToROSImageNode() override
    {
        for (int i = 0; i < nats_.count; ++i) {
            natsSubscription_Destroy(nats_.subs[i]);
        }
        natsConnection_Destroy(nats_.conn);
        nats_Close();
    }

    static void sOnTopicRaw(natsConnection*, natsSubscription*, natsMsg* msg, void* closure)
    {
        auto* ctx = static_cast<TopicCtx*>(closure);
        std::string subject = natsMsg_GetSubject(msg);
        const char* d = natsMsg_GetData(msg);
        int len = natsMsg_GetDataLength(msg);
        try { if (d && len > 0) (*ctx->fn)(subject, d, len); }
        catch (const std::exception& e) { std::cerr << "sOnTopicRaw error: " << e.what() << std::endl; }
        catch (...) { std::cerr << "sOnTopicRaw unknown error\n"; }
        natsMsg_Destroy(msg);
    }

    void publishHeartBeat()
    {
        if (!nats_.conn) return;
        json message;
        message["timeStamp"] = getCurrentTimeISO8601();
        message["status"]    = "alive";

        natsStatus s = natsConnection_PublishString(nats_.conn, "dss.dssToROSImage.heartBeat", message.dump().c_str());
        if (s != NATS_OK) {
            std::cerr << "Heartbeat publish error: " << natsStatus_GetText(s) << std::endl;
        }
        //RCLCPP_INFO(get_logger(), "publishHeartBeat");
    }

    std::string getCurrentTimeISO8601()
    {
        using namespace std::chrono;
        auto now = system_clock::now();
        auto t = system_clock::to_time_t(now);
        auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

        std::ostringstream oss;
        oss << std::put_time(std::gmtime(&t), "%Y-%m-%dT%H:%M:%S");
        oss << "." << std::setw(3) << std::setfill('0') << ms.count() << "Z";
        return oss.str();
    }



    void onTick()
    {
        publishHeartBeat();
    }

private:
    bool subscribeTopicRaw(const std::string& subject, TopicHandler handler, const char* queue = nullptr) {
        if (nats_.count >= MAX_SUBS) return false;

        topicHandlers_.emplace_back(std::make_unique<TopicHandler>(std::move(handler)));
        auto* fnPtr = topicHandlers_.back().get();

        rawCtx_.emplace_back(std::make_unique<TopicCtx>(TopicCtx{fnPtr}));
        auto* ctx = rawCtx_.back().get();

        natsSubscription* sub = nullptr;
        natsStatus s = (queue && *queue)
            ? natsConnection_QueueSubscribe(&sub, nats_.conn, subject.c_str(), queue, sOnTopicRaw, ctx)
            : natsConnection_Subscribe(&sub,      nats_.conn, subject.c_str(),       sOnTopicRaw, ctx);

        if (s != NATS_OK) {
            std::cerr << "subscribeTopicRaw failed: " << subject << " : " << natsStatus_GetText(s) << std::endl;
            rawCtx_.pop_back(); topicHandlers_.pop_back();
            return false;
        }
        nats_.subs[nats_.count++] = sub;
        return true;
    }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DSSToROSImageNode>());
    rclcpp::shutdown();
}
