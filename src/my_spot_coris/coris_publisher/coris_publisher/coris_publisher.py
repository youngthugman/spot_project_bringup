import csv
import pathlib
import math
from typing import List #allows variables to hold lists of data.
#ROS2 Imports
import rclpy #ros2 api
from rclpy.node import Node #node class
from sensor_msgs.msg import Image #used to publish sensor data as ROS messages.
from ament_index_python.packages import get_package_share_directory
# For Rviz
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class coris_publisher(Node):
    def __init__(self): #object of the class represented as self
        super().__init__('coris_publisher') #initialise the parent inherited node with a name that will set this node name to coris_publisher in the ros2 system (necessary step)
        # Declaring Parameters (config values that can be modified in param file e.g yaml). Same as declaring in innit.
        simulated_csv = pathlib.Path(get_package_share_directory('coris_publisher')) / 'data' / 'Simulation_data_CPS' #add path later
        self.declare_parameter('csv_path', str(simulated_csv))
        self.declare_parameter('topic_name', '/radiation') #maybe modify this to be more clear?
        self.declare_parameter('frame_id', 'pixel_sensor_link') # frame identificaiton for timing.
        self.declare_parameter('width', 32)
        self.declare_parameter('height', 8)
        self.declare_parameter('publish_period_s', 5.0) # hz
#        Fetching Parameter Values That Are Declared Above
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
        self.timer = self.create_timer(publish_period_s, self.publish_frame) # creates timer that calls publish frame function every publish_period_s
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
                # No range check for pixel values (allowing larger values like 20000)
                row_values = [int(cell) for cell in raw_row]
                # Append the row of pixel values to the rows list
                rows.append(row_values)
                if len(rows) != expected_height:
                    raise ValueError(
                        f'CSV must contain exactly {expected_height} non-empty rows. Found {len(rows)}.'
                    )
        # Flattening the 2D list into a 1D list
        return [value for row in rows for value in row]
    
# METHOD: Frame Publisher
    def publish_frame(self) -> None:
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = self.height
        msg.width = self.width
        msg.encoding = 'mono8'
        msg.step = self.width
        msg.data = self.pixel_data

        self.publisher.publish(msg)
        self.publish_ray_markers()

# METHOD: Pixel to angle 
    def pixel_to_angles(self,row,col):
        vertical_fov = 90
        horizontal_fov = 360
        n_rows = 8 #can use self.height 
        n_cols = 32 #can use self.width to be cleaner
        # Polar coordinate calculation.
        vertical_angle = (row * (vertical_fov / n_rows)) - (vertical_fov/2) # +- 45deg range from horizontal plane 0.
        horizontal_angle = (col * (horizontal_fov / n_cols)) - (horizontal_fov /2 ) #+_ 180 deg range from horizontal plane 0.
        # Convert angles to radians.
        vertical_angle_rad = math.radians(vertical_angle)
        horizontal_angle_rad = math.radians(horizontal_angle)
        return vertical_angle_rad, horizontal_angle_rad

# METHOD: Converts ray vectors from polar to cartesian
    def pixel_to_ray(self, row, col, cps):
        if cps <= 0: # ensure we are not plotting empty space with no data in it.
            return None #maybe have this handle a little better. 
        vertical_angle_rad, horizontal_angle_rad= self.pixel_to_angles(row, col)
        # Polar radians to cartesian.
        x = math.cos(horizontal_angle_rad) * math.cos(vertical_angle_rad)
        y = math.sin(horizontal_angle_rad) * math.cos(vertical_angle_rad)
        z = math.sin(vertical_angle_rad)
        # Ray design (m)
        ray_width = 0.01 
        ray_height = 0.01
        ray_length = 10 * ( 0.1 * cps/ 255) # cap at 10m and scale length as a function of cps. Confirm with physics later. Convert to greyscale 8 bit.
    
        return x,y,z,ray_width, ray_height, ray_length
    
# METHOD: Publish ray markers in rviz
    def publish_ray_markers(self):
        marker_array = MarkerArray()
        for row in range(self.height): 
            for col in range(self.width):
                cps = self.pixel_data[row * self.width + col] # extracts value from flattened.
                ray_data = self.pixel_to_ray(row,col,cps) #onvery extracted pixel into 3D.
                x,y,z,ray_width, ray_height, ray_length = ray_data #unpack tuple.
                # Create Marker for this ray
                marker = Marker()
                marker.header.frame_id = self.frame_id
                marker.type = Marker.LINE_STRIP  # Use line strip to represent the ray
                marker.action = Marker.ADD
                marker.scale.x = ray_width  # Ray width (fixed value)
                marker.scale.y = ray_height  # Ray height (fixed value)
                marker.color.a = 1.0  # Fully opaque
                marker.color.r = cps / 255.0  # Color based on intesity, make this a little more realistic.
                # Origin or the Ray
                start_point = Point()
                start_point.x = 0  # Sensor is at the origin
                start_point.y = 0
                start_point.z = 0
                # End of Ray
                end_point = Point()
                end_point.x = x  # Projected 3D point
                end_point.y = y
                end_point.z = z
                marker.points.append(start_point)
                marker.points.append(end_point)
                marker_array.markers.append(marker)
            # Publish the marker array
            self.marker_publisher.publish(marker_array)

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