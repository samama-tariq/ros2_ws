# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# import cv2
# import numpy as np

# class CameraPublisher(Node):

#     def __init__(self):
#         super().__init__('camera_publisher')
#         self.publisher_ = self.create_publisher(Image, 'drone/camera_feed', 10)
#         self.cap = cv2.VideoCapture('rtsp://192.168.144.25:8554/main.264')  # Replace with your RTSP URL

#     def run(self):
#         while rclpy.ok():
#             ret, frame = self.cap.read()
#             if ret:
#                 # Convert the OpenCV frame to ROS Image message
#                 image_message = self.convert_opencv_to_imgmsg(frame, "bgr8")
#                 self.publisher_.publish(image_message)

#     def convert_opencv_to_imgmsg(self, cv_image, encoding):
#         # Convert OpenCV image to ROS Image message format
#         msg = Image()
#         msg.height = cv_image.shape[0]
#         msg.width = cv_image.shape[1]
#         msg.encoding = encoding
#         msg.data = cv_image.tobytes()
#         return msg

# def main(args=None):
#     rclpy.init(args=args)
#     cam_pub = CameraPublisher()
#     cam_pub.run()
#     cam_pub.cap.release()
#     cam_pub.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
  
class ImagePublisher(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_publisher')
       
    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
       
    # We will publish a message every 0.1 seconds
    timer_period = 0.001  # seconds
       
    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)
          
    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    self.cap = cv2.VideoCapture('rtsp://192.168.144.25:8554/main.264')
          
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
    
  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.1 seconds.
    """
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    ret, frame = self.cap.read()
           
    if ret == True:
      # Publish the image.
      # The 'cv2_to_imgmsg' method converts an OpenCV
      # image to a ROS 2 image message
      self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
  
    # Display the message on the console
    self.get_logger().info('Publishing video frame')
   
def main(args=None):
   
  # Initialize the rclpy library
  rclpy.init(args=args)
   
  # Create the node
  image_publisher = ImagePublisher()
   
  # Spin the node so the callback function is called.
  rclpy.spin(image_publisher)
   
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_publisher.destroy_node()
   
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
   
if __name__ == '__main__':
  main()
