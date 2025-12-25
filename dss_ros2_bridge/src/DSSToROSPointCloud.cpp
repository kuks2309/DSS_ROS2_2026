#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
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


class DSSToROSPointCloudNode : public rclcpp::Node
{
public:
    using TopicHandler  = std::function<void(const std::string& subject, const char* data, int len)>;
    struct TopicCtx     { TopicHandler*     fn; };

    // NATS
    NatsClient nats_;

    // 수명 보장 컨테이너
    
    std::vector<std::unique_ptr<TopicHandler>>                  topicHandlers_;
    std::vector<std::unique_ptr<TopicCtx>>                      rawCtx_;
    rclcpp::TimerBase::SharedPtr                                timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;


    sensor_msgs::msg::PointCloud2 createPointCloud(const dss::DssLidarPointCloud& pcd_msg)
    {
        double stamp_sec = pcd_msg.header().stamp();
        
        const uint32_t num_points = pcd_msg.width();
        const uint8_t* raw = reinterpret_cast<const uint8_t*>(pcd_msg.data().data());
        const uint32_t step = pcd_msg.point_step();
        const float max_range = 100.0f;

        sensor_msgs::msg::PointCloud2 msg;

        // ROS 메시지에 채우기
        rclcpp::Time ros_stamp(static_cast<int64_t>(stamp_sec * 1e9),RCL_ROS_TIME);        

        

        //RCLCPP_INFO(rclcpp::get_logger("lidar_bridge"),"lidar stamp = %ld.%09u",ros_stamp.sec,ros_stamp.nanosec);        


        msg.header.stamp = ros_stamp;
        msg.header.frame_id = "map";


        //RCLCPP_INFO(rclcpp::get_logger("lidar_bridge"),"lidar stamp = %ld.%09u",msg.header.stamp.sec,msg.header.stamp.nanosec);        

        msg.height = 1;
        msg.width = num_points;

        msg.is_bigendian = false;
        msg.point_step = 16;   // float32 x,y,z,intensity
        msg.row_step = msg.point_step * msg.width;

        msg.is_dense = true;

        // 필드 정의
        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2Fields(
            4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32
        );

        modifier.resize(num_points);

        // 데이터 채우기 (디코딩만 적용)
        sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_i(msg, "intensity");

        for (size_t i = 0; i < num_points; i++)
        {
            const uint8_t* ptr = raw + i * step;

            float x = 0.0f, y = 0.0f, z = 0.0f, intensity = 0.0f;

            if (step == 16)
            {
                // float32 <ffff>
                x = *reinterpret_cast<const float*>(ptr + 0);
                y = *reinterpret_cast<const float*>(ptr + 4);
                z = *reinterpret_cast<const float*>(ptr + 8);
                intensity = *reinterpret_cast<const float*>(ptr + 12);
            }
            else if (step == 8)
            {
                // int16 quantized <hhhh>
                int16_t xi = *reinterpret_cast<const int16_t*>(ptr + 0);
                int16_t yi = *reinterpret_cast<const int16_t*>(ptr + 2);
                int16_t zi = *reinterpret_cast<const int16_t*>(ptr + 4);
                int16_t ii = *reinterpret_cast<const int16_t*>(ptr + 6);

                x = (xi / 32767.0f) * max_range;
                y = (yi / 32767.0f) * max_range;
                z = (zi / 32767.0f) * max_range;
                intensity = (ii / 32767.0f);
            }

            else if (step == 10)
            {
                // int16 quantized + ring <hhhhh>
                int16_t xi = *reinterpret_cast<const int16_t*>(ptr + 0);
                int16_t yi = *reinterpret_cast<const int16_t*>(ptr + 2);
                int16_t zi = *reinterpret_cast<const int16_t*>(ptr + 4);
                int16_t ii = *reinterpret_cast<const int16_t*>(ptr + 6);

                x = (xi / 32767.0f) * max_range;
                y = (yi / 32767.0f) * max_range;
                z = (zi / 32767.0f) * max_range;
                intensity = (ii / 32767.0f);
            }

            else if (step == 14)
            {
                // int16 quantized + ring <hhhhh>
                int16_t xi = *reinterpret_cast<const int16_t*>(ptr + 0);
                int16_t yi = *reinterpret_cast<const int16_t*>(ptr + 2);
                int16_t zi = *reinterpret_cast<const int16_t*>(ptr + 4);
                int16_t ii = *reinterpret_cast<const int16_t*>(ptr + 6);

                x = (xi / 32767.0f) * max_range;
                y = (yi / 32767.0f) * max_range;
                z = (zi / 32767.0f) * max_range;
                intensity = (ii / 32767.0f);
            }


            *iter_x = x;
            *iter_y = y;
            *iter_z = z;
            *iter_i = intensity;

            ++iter_x; ++iter_y; ++iter_z; ++iter_i;
        }

        return msg;
    }


    float dequantize(int16_t QuantizedValue, float MaxAbs)
    {
        return (static_cast<float>(QuantizedValue) / 32767.0f) * MaxAbs;
    }


    sensor_msgs::msg::PointCloud2 createPointCloud2(const dss::DssLidarPointCloud& pcd_msg)
    {
        double stamp_sec = pcd_msg.header().stamp();

        const uint32_t num_points = pcd_msg.width();
        const uint8_t* raw = reinterpret_cast<const uint8_t*>(pcd_msg.data().data());
        const uint32_t step = pcd_msg.point_step();

        sensor_msgs::msg::PointCloud2 msg;

        // -------------------------------------------------
        // sim_time 기반 stamp
        // -------------------------------------------------
        rclcpp::Time ros_stamp(static_cast<int64_t>(stamp_sec * 1e9),RCL_ROS_TIME);

        msg.header.stamp = ros_stamp;
        msg.header.frame_id = "lidar_link";

        /*
        RCLCPP_INFO(
            rclcpp::get_logger("lidar_bridge"),
            "lidar stamp = %ld.%09u",
            msg.header.stamp.sec,
            msg.header.stamp.nanosec
        );
        */

        // -------------------------------------------------
        // PointCloud2 기본 설정
        // -------------------------------------------------
        msg.height = 1;
        msg.width  = num_points;
        msg.is_bigendian = false;
        msg.is_dense = true;

        // x y z intensity ring time
        msg.point_step = 20;                 // 4*4 + 2 + padding + 4
        msg.row_step   = msg.point_step * msg.width;

        // -------------------------------------------------
        // 필드 정의 (LIO-SAM 친화)
        // -------------------------------------------------
        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2Fields(
            6,
            "x",         1, sensor_msgs::msg::PointField::FLOAT32,
            "y",         1, sensor_msgs::msg::PointField::FLOAT32,
            "z",         1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
            "ring",      1, sensor_msgs::msg::PointField::UINT16,
            "time",      1, sensor_msgs::msg::PointField::FLOAT32
        );

        modifier.resize(num_points);

        // -------------------------------------------------
        // Iterators
        // -------------------------------------------------
        sensor_msgs::PointCloud2Iterator<float>   iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float>   iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float>   iter_z(msg, "z");
        sensor_msgs::PointCloud2Iterator<float>   iter_i(msg, "intensity");
        sensor_msgs::PointCloud2Iterator<uint16_t> iter_r(msg, "ring");
        sensor_msgs::PointCloud2Iterator<float>   iter_t(msg, "time");

        // -------------------------------------------------
        // 데이터 디코딩
        // (Unreal → Protobuf → ROS2)
        // -------------------------------------------------
        for (uint32_t i = 0; i < num_points; i++)
        {
            const uint8_t* ptr = raw + i * step;

            int16_t xi = *reinterpret_cast<const int16_t*>(ptr + 0);
            int16_t yi = *reinterpret_cast<const int16_t*>(ptr + 2);
            int16_t zi = *reinterpret_cast<const int16_t*>(ptr + 4);
            int16_t ii = *reinterpret_cast<const int16_t*>(ptr + 6);
            int16_t ri = *reinterpret_cast<const int16_t*>(ptr + 8);
            float   ti = *reinterpret_cast<const float*>(ptr + 10);

            // Quantized → meter
            //constexpr float POS_SCALE = 100.0f;   // Unreal에서 쓰던 값
            //constexpr float INT_SCALE = 1.0f;

            
            
            float x = dequantize(xi,100);
            float y = dequantize(yi,100);
            float z = dequantize(zi,100);
            uint16_t r = ri;
            float intensity = dequantize(ii,1);

            *iter_x = x;
            *iter_y = y;
            *iter_z = z;
            *iter_i = intensity;
            *iter_r = static_cast<uint16_t>(ri);
            *iter_t = ti;

            ++iter_x; ++iter_y; ++iter_z;
            ++iter_i; ++iter_r; ++iter_t;
        }

        return msg;
    }

    


public:
    DSSToROSPointCloudNode() : Node("DSSToROSPointCloudNode") {
        //this->declare_parameter<std::string>("nats_server", "nats://127.0.0.1:4222");
        //std::string kNatsUrl = this->get_parameter("nats_server").as_string();
        std::string kNatsUrl = "nats://" + getDefaultGateway()+ ":4222";
        RCLCPP_INFO(get_logger(), kNatsUrl.c_str());
        natsStatus s = natsConnection_ConnectTo(&nats_.conn, kNatsUrl.c_str());

        if (s != NATS_OK) {
            std::cerr << "NATS connect failed: " << natsStatus_GetText(s) << std::endl;
            return;
        }     
        timer_   = this->create_wall_timer(std::chrono::seconds(3), std::bind(&DSSToROSPointCloudNode::onTick, this));
        subscribeTopicRaw("dss.sensor.lidar",
            [this](const std::string& subject, const char* bytes, int len)
            {
                dss::DssLidarPointCloud pcd_msg;
                if (!pcd_msg.ParseFromArray(bytes, len)) {
                    std::cerr << "Failed to parse DSSImage protobuf message\n";
                    return; 
                }
                pub_->publish(createPointCloud2(pcd_msg));
            }
        );
        //pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/dss/sensor/lidar", 10);
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points", 10);
        RCLCPP_INFO(get_logger(), "[NATS]dss.sensor.lidar → [ROS2]/dss/sensor/lidar");
    }

    ~DSSToROSPointCloudNode() override {
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
    
        natsStatus s = natsConnection_PublishString(nats_.conn,"dss.dssToROSPointCloud.heartBeat", message.dump().c_str());
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

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DSSToROSPointCloudNode>());
    rclcpp::shutdown();
}
