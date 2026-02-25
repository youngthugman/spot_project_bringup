import csv
import pathlib
from typing import List #allows variables to hold lists of data.
#ROS2 Imports
import rclpy #ros2 api
from rclpy.node import Node #node class
from sensor_msgs.msg import Image #used to publish sensor data as ROS messages.
from ament_index_python.packages import get_package_share_directory

class coris_publisher(Node):
    def __innit__(self): #object of the class represented as self
        super().__init__('coris_publisher') #initialise the parent inherited node with a name that will set this node name to coris_publisher in the ros2 system (necessary step)

# Declaring Parameters (config values that can be modified in param file e.g yaml). Same as declaring in innit.
        simulated_csv = pathlib.Path(get_package_share_directory('')) #add path later
        self.declare_parameter('csv_path', str(simulated_csv))
        self.declare_parameter('topic_name', '/sim/pixels/image') #maybe modify this to be more clear?
        self.declare_parameter('frame_id', 'pixel_sensor_link') # frame identificaiton for timing.
        self.declare_parameter('width', 32)
        self.declare_parameter('height', 8)
        self.declare_parameter('publish_period_s', 5.0) # hz
# Fetching Parameter Values That Are Declared Above
        self.csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        publish_period_s = self.get_parameter('publish_period_s').get_parameter_value().double_value
# Catch Errors for Image Width and Height
        if self.width <= 0 or self.height <= 0:
            raise ValueError('width and height must be > 0')
        if publish_period_s <= 0.0:
            raise ValueError('publish_period_s must be > 0')
# Loading Pixel Simulation Data using our method (remember method is a function in a class)
        self.pixel_data = self._load_pixel_csv(self.csv_path, self.height, self.width)
# Publisher Set Up
        self.publisher = self.create_publisher(Image, self.topic_name, 10) # publishes messsages of type image to topic name x.
        self.timer = self.create_timer(publish_period_s, self._publish_frame) # creates timer that calls publish frame function every publish_period_s
# Logger Message
        self.get_logger().info(
            f'Publishing {self.height}x{self.width} pixel frame from {self.csv_path} '
            f'to {self.topic_name} every {publish_period_s:.2f}s'
        ) 
# METHOD: Load Pixels from csv 
    def _load_pixel_csv(self, csv_path: str, expected_height: int, expected_width: int) -> List[int]:
        path = pathlib.Path(csv_path)
        if not path.exists():
            raise FileNotFoundError(f'CSV path does not exist: {csv_path}')
        rows: List[List[int]] = []
        with path.open('r', newline='') as handle:
            reader = csv.reader(handle)
            for raw_row in reader:
                if not raw_row or all(cell.strip() == '' for cell in raw_row):
                    continue
                if len(raw_row) != expected_width:
                    raise ValueError(
                        f'Each CSV row must have {expected_width} columns. Found {len(raw_row)}.'
                    )
                row_values = [int(cell) for cell in raw_row]
                for value in row_values:
                    if value < 0 or value > 255:
                        raise ValueError('Pixel values must be in [0, 255] for mono8 image encoding.')
                rows.append(row_values)
                if len(rows) != expected_height:
                    raise ValueError(
                        f'CSV must contain exactly {expected_height} non-empty rows. Found {len(rows)}.'
                    )        
        return [value for row in rows for value in row]
    
# METHOD: Frame Publisher
    def _publish_frame(self) -> None:
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = self.height
        msg.width = self.width
        msg.encoding = 'mono8'
        msg.step = self.width
        msg.data = self.pixel_data

        self.publisher.publish(msg)

# Main Function
def main(args=None) -> None:
    rclpy.init(args=args)
    node = coris_publisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()