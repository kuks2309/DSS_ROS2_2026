/*
    DSS To ROS GPS
*/
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
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

class DSSToROSGpsNode : public rclcpp::Node
{
public:
    using TopicHandler  = std::function<void(const std::string& subject, const char* data, int len)>;
    struct TopicCtx     { TopicHandler*     fn; };

    // NATS
    NatsClient nats_;

    // 수명 보장 컨테이너
    
    std::vector<std::unique_ptr<TopicHandler>>          topicHandlers_;
    std::vector<std::unique_ptr<TopicCtx>>              rawCtx_;
    rclcpp::TimerBase::SharedPtr                        timer_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;

    sensor_msgs::msg::NavSatFix CreateGPSTopic(const dss::DSSGPS& dss_gps)
    {
        double stamp_sec = dss_gps.header().stamp();
        sensor_msgs::msg::NavSatFix ros_gps;
        // Header
        rclcpp::Time ros_stamp(static_cast<int64_t>(stamp_sec * 1e9),RCL_ROS_TIME);
        ros_gps.header.stamp = ros_stamp;
        ros_gps.header.frame_id = dss_gps.header().frame_id();

        //RCLCPP_INFO(rclcpp::get_logger("gps_bridge"),"gps stamp = %ld.%09u",ros_gps.header.stamp.sec,ros_gps.header.stamp.nanosec);        

        // Status
        ros_gps.status.status = dss_gps.status().status();
        ros_gps.status.service = dss_gps.status().service();

        // Position
        ros_gps.latitude  = dss_gps.latitude();
        ros_gps.longitude = dss_gps.longitude();
        ros_gps.altitude  = dss_gps.altitude();

        // Covariance
        if (dss_gps.position_covariance_size() == 9)
        {
            for (int i = 0; i < 9; ++i)
            {
                ros_gps.position_covariance[i] =
                    dss_gps.position_covariance(i);
            }
        }

        ros_gps.position_covariance_type =
            dss_gps.position_covariance_type();

        //RCLCPP_INFO(get_logger(), "[NATS]dss.sensor.gps → [ROS2]/dss/sensor/gps");
        
        return ros_gps;
    }
public:
    DSSToROSGpsNode() : Node("DSSToROSGpsNode") {
        //RCLCPP_INFO(get_logger(),get_default_gateway().c_str());
        //RCLCPP_INFO(get_logger(),get_default_gateway().c_str());
        RCLCPP_INFO(get_logger(),getDefaultGateway().c_str());        

        //this->declare_parameter<std::string>("nats_server", "nats://127.0.0.1:4222");
        //std::string kNatsUrl = this->get_parameter("nats_server").as_string();
         std::string kNatsUrl = "nats://" + getDefaultGateway()+ ":4222";
        RCLCPP_INFO(get_logger(), kNatsUrl.c_str());
        natsStatus s = natsConnection_ConnectTo(&nats_.conn, kNatsUrl.c_str());
        if (s != NATS_OK) {
            std::cerr << "NATS connect failed: " << natsStatus_GetText(s) << std::endl;
            return;
        }     
        
        timer_   = this->create_wall_timer(std::chrono::seconds(3), std::bind(&DSSToROSGpsNode::onTick, this));
        subscribeTopicRaw("dss.sensor.gps",
            [this](const std::string& subject, const char* bytes, int len)
            {
                dss::DSSGPS gps_msg;
                if (!gps_msg.ParseFromArray(bytes, len)) {
                    std::cerr << "Failed to parse DSSGPS protobuf message\n";
                    return; 
                }
                pub_->publish(CreateGPSTopic(gps_msg));
            }
        );
        pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/dss/sensor/gps/fix", 10);

        RCLCPP_INFO(get_logger(), "[NATS]dss.sensor.gps → [ROS2]/dss/sensor/gps/fix");
    }

    ~DSSToROSGpsNode() override {
        for (int i = 0; i < nats_.count; ++i) {
            natsSubscription_Destroy(nats_.subs[i]);
        }
        natsConnection_Destroy(nats_.conn);
        nats_Close();
    }

    static void sOnTopicRaw(natsConnection*, natsSubscription*, natsMsg* msg, void* closure) {
        auto* ctx = static_cast<TopicCtx*>(closure);
        std::string subject = natsMsg_GetSubject(msg);
        const char* d = natsMsg_GetData(msg);
        int len = natsMsg_GetDataLength(msg);
        try { if (d && len > 0) (*ctx->fn)(subject, d, len); }
        catch (const std::exception& e) { std::cerr << "sOnTopicRaw error: " << e.what() << std::endl; }
        catch (...) { std::cerr << "sOnTopicRaw unknown error\n"; }
        natsMsg_Destroy(msg);
    }

    void publishHeartBeat() {
        if (!nats_.conn) return;
        json message;
        message["timeStamp"] = getCurrentTimeISO8601();
        message["status"]    = "alive";

        natsStatus s = natsConnection_PublishString(nats_.conn, "dss.dssToROSGps.heartBeat", message.dump().c_str());
        if (s != NATS_OK) {
            std::cerr << "Heartbeat publish error: " << natsStatus_GetText(s) << std::endl;
        }
    }

    std::string getCurrentTimeISO8601() {
        using namespace std::chrono;
        auto now = system_clock::now();
        auto t = system_clock::to_time_t(now);
        auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

        std::ostringstream oss;
        oss << std::put_time(std::gmtime(&t), "%Y-%m-%dT%H:%M:%S");
        oss << "." << std::setw(3) << std::setfill('0') << ms.count() << "Z";
        return oss.str();
    }



    void onTick() {
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
    rclcpp::spin(std::make_shared<DSSToROSGpsNode>());
    rclcpp::shutdown();
}
