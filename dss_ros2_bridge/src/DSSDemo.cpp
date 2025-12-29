#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "DSS.VSSClient.h"

class DssDemoController : public rclcpp::Node
{
public:
    DssDemoController() : Node("DssDemoController") {
        RCLCPP_INFO(get_logger(), "DSS Demo Controller Started");

        //VSS 초기화
        auto& vss = DSSVssClient::singleton();
        vss.start("172.25.96.1",8886,4222);


        // ROS2 IMU 구독
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/dss/sensor/imu",
            10,
            [this](sensor_msgs::msg::Imu::SharedPtr msg) 
            {
                this->last_imu_ = *msg;
            }
        );

        // ROS2 PCD 구독
        pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/dss/sensor/lidar",
            10,
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) 
            {
                this->last_cloud_ = *msg;
            }
        );

        // ROS2 Image 구독
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/dss/sensor/camera/rgb",
            10,
            [this](sensor_msgs::msg::Image::SharedPtr msg) 
            {
                this->last_image_ = *msg;
            }
        );        
        
      
        //VSS 클라이언트 사용 예제
        // 구독
        natsStatus subStatus;
        auto* sub = vss.subscribe("vss.sensor.speed",[](const std::string& subject, const std::string& payload)
            {
                //std::cout << "[SUB] " << subject << " => " << payload << std::endl;
                std::atof(payload.c_str());
            },
            &subStatus
        );


        // Get 예시 (비동기)
        vss.get("Vehicle.Cabin.Tailgate.Position", [](natsStatus st, const std::string& reply)
            {
                if (st == NATS_OK)
                    std::cout << "[GET] Vehicle.Cabin.Tailgate.Position = " << reply << std::endl;
                else
                    std::cerr << "[GET] 실패: " << natsStatus_GetText(st) << std::endl;
            }
        );

        // Set 예시
        vss.set("Vehicle.Cabin.Door.Row1.DriverSide.Switch", "open", [](natsStatus st)
            {
                if (st == NATS_OK)
                    std::cout << "[SET] 성공" << std::endl;
                else
                    std::cerr << "[SET] 실패: " << natsStatus_GetText(st) << std::endl;
            }
        );

        // Set 예시
        vss.set("Vehicle.Cabin.Door.Row1.PassengerSide.Switch", "open", [](natsStatus st)
            {
                if (st == NATS_OK)
                    std::cout << "[SET] 성공" << std::endl;
                else
                    std::cerr << "[SET] 실패: " << natsStatus_GetText(st) << std::endl;
            }
        );


    
        // Set 예시
        vss.set("Vehicle.Cabin.Door.Row2.DriverSide.Switch", "open", [](natsStatus st)
            {
                if (st == NATS_OK)
                    std::cout << "[SET] 성공" << std::endl;
                else
                    std::cerr << "[SET] 실패: " << natsStatus_GetText(st) << std::endl;
            }
        );


        // Set 예시
        vss.set("Vehicle.Cabin.Door.Row2.PassengerSide.Switch", "open", [](natsStatus st)
            {
                if (st == NATS_OK)
                    std::cout << "[SET] 성공" << std::endl;
                else
                    std::cerr << "[SET] 실패: " << natsStatus_GetText(st) << std::endl;
            }
        );

        
        // Set 예시
        vss.set("Vehicle.Cabin.Tailgate.Switch", "open", [](natsStatus st)
            {
                if (st == NATS_OK)
                    std::cout << "[SET] 성공" << std::endl;
                else
                    std::cerr << "[SET] 실패: " << natsStatus_GetText(st) << std::endl;
            }
        );

        // Set 예시
        vss.set("Vehicle.Cabin.Sunroof.Switch", "open", [](natsStatus st)
            {
                if (st == NATS_OK)
                    std::cout << "[SET] 성공" << std::endl;
                else
                    std::cerr << "[SET] 실패: " << natsStatus_GetText(st) << std::endl;
            }
        );


        // 차량 제어
        auto send_rate_ = 50; // 20 Hz
        control_timer_ = create_wall_timer(std::chrono::milliseconds(1000 / send_rate_),
            [this]()
            {
                auto& vss = DSSVssClient::singleton();
                vss.setDriveControl(0.0f, 0.0f, 0.0f); // 일단 제어 초기화
            }     
        );

    }

private:
    // ---- Sensor data storage ----
    sensor_msgs::msg::Imu last_imu_;
    sensor_msgs::msg::PointCloud2 last_cloud_;
    sensor_msgs::msg::Image last_image_;

    // ---- ROS Entities ----
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DssDemoController>());
    rclcpp::shutdown();
    return 0;
}
