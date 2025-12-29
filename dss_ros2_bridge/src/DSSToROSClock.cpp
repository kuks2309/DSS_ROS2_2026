/*
    ros2 topic echo /clock
*/
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
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

class DSSToROSClockNode : public rclcpp::Node
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
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_;

    //uint64_t sim_time_ns_ = 0;

    
    rosgraph_msgs::msg::Clock createClock(const dss::DssOneFrameFixedRateResult& msg)
    {
        // delta는 항상 고정값 (0.005)
        constexpr uint64_t DT_NS = 5'000'000ULL; // 0.005 sec
        const uint64_t sim_time_ns = static_cast<uint64_t>(msg.frame_count()) * DT_NS;
        rosgraph_msgs::msg::Clock clock_msg;
        clock_msg.clock.sec = static_cast<int32_t>(sim_time_ns / 1'000'000'000ULL);
        clock_msg.clock.nanosec = static_cast<uint32_t>(sim_time_ns % 1'000'000'000ULL);

        //RCLCPP_INFO(rclcpp::get_logger("clock"),"Clock stamp = %ld.%09u",clock_msg.clock.sec,clock_msg.clock.nanosec); 

        return clock_msg;
    }

public:
    DSSToROSClockNode() : Node("DSSToROSClockNode") {
        std::string kNatsUrl = "nats://" + getDefaultGateway()+ ":4222";
        RCLCPP_INFO(get_logger(), kNatsUrl.c_str());
        natsStatus s = natsConnection_ConnectTo(&nats_.conn, kNatsUrl.c_str());
        if (s != NATS_OK) {
            std::cerr << "NATS connect failed: " << natsStatus_GetText(s) << std::endl;
            return;
        }     
        
        timer_   = this->create_wall_timer(std::chrono::seconds(3), std::bind(&DSSToROSClockNode::onTick, this));
        subscribeTopicRaw("dss.simTime.clock",
            [this](const std::string& subject, const char* bytes, int len)
            {
                
                dss::DssOneFrameFixedRateResult msg;
                if (!msg.ParseFromArray(bytes, len)) {
                    std::cerr << "Failed to parse DssOneFrameFixedRateResult protobuf message\n";
                }
                
                auto use_sim_time_ = this->get_parameter("use_sim_time").as_bool();
                if (use_sim_time_){
                    pub_->publish(createClock(msg));
                    //RCLCPP_INFO(get_logger(), "[NATS]dss.sensor.clock → [ROS2]/clock");
                }
            }
        );
        pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        RCLCPP_INFO(get_logger(), "[NATS]dss.simTime.clock → [ROS2]/clock");
    }

    ~DSSToROSClockNode() override {
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

        natsStatus s = natsConnection_PublishString(nats_.conn, "dss.dssToROSClock.heartBeat", message.dump().c_str());
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
        /*
        auto use_sim_time_ = this->get_parameter("use_sim_time").as_bool();
        if (!use_sim_time_) return;
        pub_->publish(createDummyClock());
        RCLCPP_INFO(get_logger(), "[NATS]dss.sensor.clock → [ROS2]/clock");
        */
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
    rclcpp::spin(std::make_shared<DSSToROSClockNode>());
    rclcpp::shutdown();
}
