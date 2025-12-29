import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, PointCloud2, Image
import asyncio
from dss_controller.dss_vss_client import DSSVssClient


class DSSController(Node):
    def __init__(self):
        super().__init__('dss_controller')

        self.get_logger().info("DSS Demo Controller Started")

        # Python VSS Client
        self.client = DSSVssClient.singleton()
        self.loop = asyncio.get_event_loop()
        self.client.start(ip="172.26.160.1",drive_port=8886, vss_port=4222)

        self.imu_sub = self.create_subscription(
            Imu,
            "/dss/sensor/imu",
            lambda msg: self._on_imu(msg),
            10
        )

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            "/dss/sensor/lidar",
            lambda msg: self._on_cloud(msg),
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            "/dss/sensor/camera/rgb",
            lambda msg: self._on_image(msg),
            10
        )
        
        self.test_get()
        self.test_set()
        
        # ============ Control Timer ============
        send_rate = 50  # 50Hz
        self.control_timer = self.create_timer(1.0 / send_rate, self._on_control_timer)
        
    def test_set(self):
        self.client.set("Vehicle.Cabin.Door.Row1.DriverSide.Switch", "Open")
    
    def test_get(self):
        future = self.client.get("Vehicle.Cabin.Tailgate.Position")
        reply, status = future.result()

        if status == "OK":
            print(f"[GET] Tailgate.Position = {reply}")
        else:
            print(f"[GET] 실패: {status}")        

    # -------------------------------------------------------
    # ROS2 Callbacks
    # -------------------------------------------------------
    def _on_imu(self, msg: Imu):
        self.last_imu = msg
        print(f"[DSSVssClient] imu")

    def _on_cloud(self, msg: PointCloud2):
        self.last_cloud = msg
        print(f"[DSSVssClient] pcd")

    def _on_image(self, msg: Image):
        self.last_image = msg
        print(f"[DSSVssClient] image")

    # -------------------------------------------------------
    # Control Loop (50Hz)
    # -------------------------------------------------------
    def _on_control_timer(self):
        # throttle, steer, brake
        self.client.set_drive_control(1.0, 0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = DSSController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
