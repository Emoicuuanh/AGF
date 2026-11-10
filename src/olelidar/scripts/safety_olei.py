import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8
import numpy as np

class LidarReader:
    def __init__(self, topic_name='/scan', angle_min=-30.0, angle_max=30.0, publish_topic='/scan_olei_safety'):
        # Khởi tạo node ROS
        rospy.init_node('olei_safety_node', anonymous=True)

        # Topic tên topic để subscribe
        self.topic_name = topic_name

        # Phạm vi góc mong muốn
        self.angle_min = angle_min  # Góc bắt đầu (đơn vị: độ)
        self.angle_max = angle_max  # Góc kết thúc (đơn vị: độ)

        # Đăng ký callback với topic LiDAR
        self.lidar_data = None
        self.data_safety = Int8()
        self.subscriber = rospy.Subscriber(self.topic_name, LaserScan, self.lidar_callback)

        # Khởi tạo publisher để gửi dữ liệu đã lọc
        self.publisher = rospy.Publisher(publish_topic, LaserScan, queue_size=10)

        self.pub_safety_olei = rospy.Publisher("/safety_olei", Int8, queue_size=1)

    def lidar_callback(self, data):
        """
        Callback function xử lý dữ liệu LiDAR
        """
        self.lidar_data = data
        # rospy.loginfo("Received LiDAR Data")

        # Lọc dữ liệu LiDAR theo phạm vi góc mong muốn
        self.process_lidar_data()

    def process_lidar_data(self):
        self.data_safety.data = 1
        """
        Xử lý và publish dữ liệu LiDAR đã lọc trong phạm vi góc đã cho
        """
        if self.lidar_data is not None:
            # Tính toán chỉ số bắt đầu và kết thúc trong mảng `ranges` dựa trên góc
            angle_min = self.lidar_data.angle_min
            angle_max = self.lidar_data.angle_max
            angle_increment = self.lidar_data.angle_increment

            # Chuyển góc từ độ sang radian
            angle_min_rad = np.deg2rad(self.angle_min)
            angle_max_rad = np.deg2rad(self.angle_max)

            # Tính chỉ số bắt đầu và kết thúc trong mảng `ranges`
            start_index = int((angle_min_rad - angle_min) / angle_increment)
            end_index = int((angle_max_rad - angle_min) / angle_increment)

            # Lọc dữ liệu từ mảng `ranges` dựa trên phạm vi góc
            filtered_ranges = self.lidar_data.ranges[start_index:end_index]

            for distance in filtered_ranges:
                if distance < 0.07 and distance != 0.0:
                    print(distance)
                    self.data_safety.data = 0

            self.pub_safety_olei.publish(self.data_safety)
            # rospy.loginfo(f"Filtered ranges from {self.angle_min}° to {self.angle_max}°")
            # rospy.loginfo(f"Filtered ranges: {filtered_ranges}")

            # Tạo một thông điệp LaserScan mới để publish dữ liệu đã lọc
            filtered_scan = LaserScan()
            filtered_scan.header = self.lidar_data.header
            filtered_scan.header.stamp = rospy.Time.now()  # Cập nhật thời gian
            filtered_scan.angle_min = angle_min_rad
            filtered_scan.angle_max = angle_max_rad
            filtered_scan.angle_increment = self.lidar_data.angle_increment
            filtered_scan.time_increment = self.lidar_data.time_increment
            filtered_scan.scan_time = self.lidar_data.scan_time
            filtered_scan.range_min = self.lidar_data.range_min
            filtered_scan.range_max = self.lidar_data.range_max

            # Cập nhật các giá trị range đã lọc vào thông điệp
            filtered_scan.ranges = filtered_ranges
            filtered_scan.intensities = self.lidar_data.intensities[start_index:end_index]  # Nếu có dữ liệu intensities

            # Publish dữ liệu đã lọc
            self.publisher.publish(filtered_scan)
            # rospy.loginfo(f"Published filtered LiDAR data to {self.publisher.name}")


if __name__ == '__main__':
    # Khởi tạo và chạy chương trình LidarReader
    rospy.loginfo("Init node Safety Olei")
    lidar_reader = LidarReader('/scan_1', angle_min=-110.0, angle_max=30.0, publish_topic='/scan_3')  # Phạm vi góc cần lọc và topic publish
    rospy.spin()
