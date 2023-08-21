# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# import cv2
# import numpy as np

# class VideoSubscriber(Node):

#     def __init__(self):
#         super().__init__('video_subscriber')
#         self.subscription = self.create_subscription(
#             Image,
#             'video_frames',
#             self.listener_callback,
#             10)

#     def listener_callback(self, msg):
#         cv_image = self.convert_imgmsg_to_opencv(msg)
#         cv2.imshow('Drone Feed', cv_image)
#         cv2.waitKey(30)  # 1 millisecond delay to refresh the window

#     def convert_imgmsg_to_opencv(self, img_msg):
#         # Convert ROS Image message to OpenCV image
#         dtype = np.dtype(np.uint8)
#         dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
#         image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3),
#                                   dtype=dtype, buffer=img_msg.data)
#         return image_opencv

# def main(args=None):
#     rclpy.init(args=args)
#     video_sub = VideoSubscriber()
#     rclpy.spin(video_sub)
#     video_sub.destroy_node()
#     cv2.destroyAllWindows()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# import rclpy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import image_transport

# class VideoSubscriber(Node):

#     def __init__(self):
#         super().__init__('video_subscriber')
#         self.bridge = CvBridge()
#         self.image_transport_ = image_transport.ImageTransport(self)
#         self.subscriber_ = self.image_transport_.subscribe('video_frames', self.listener_callback, 10)

#     def listener_callback(self, msg):
#         cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         cv2.imshow('Drone Feed', cv_image)
#         cv2.waitKey(1)  # 1 millisecond delay to refresh the window

# def main(args=None):
#     rclpy.init(args=args)
#     video_sub = VideoSubscriber()
#     rclpy.spin(video_sub)
#     video_sub.destroy_node()
#     cv2.destroyAllWindows()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# working fine

# import rclpy # Python library for ROS 2
# from rclpy.node import Node # Handles the creation of nodes
# from sensor_msgs.msg import Image # Image is the message type
# from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
# import cv2 # OpenCV library
  
# class ImageSubscriber(Node):
#   """
#   Create an ImageSubscriber class, which is a subclass of the Node class.
#   """
#   def __init__(self):
#     """
#     Class constructor to set up the node
#     """
#     # Initiate the Node class's constructor and give it a name
#     super().__init__('image_subscriber')
       
#     # Create the subscriber. This subscriber will receive an Image
#     # from the video_frames topic. The queue size is 10 messages.
#     self.subscription = self.create_subscription(
#       Image, 
#       'video_frames', 
#       self.listener_callback, 
#       10)
#     self.subscription # prevent unused variable warning
       
#     # Used to convert between ROS and OpenCV images
#     self.br = CvBridge()
    
#   def listener_callback(self, data):
#     """
#     Callback function.
#     """
#     # Display the message on the console
#     self.get_logger().info('Receiving video frame')
#     print("Callback triggered!")
  
#     # Convert ROS Image message to OpenCV image
#     current_frame = self.br.imgmsg_to_cv2(data)
     
#     # Display image
#     cv2.imshow("camera", current_frame)
     
#     cv2.waitKey(1)
   
# def main(args=None):
   
#   # Initialize the rclpy library
#   rclpy.init(args=args)
   
#   # Create the node
#   image_subscriber = ImageSubscriber()
   
#   # Spin the node so the callback function is called.
#   rclpy.spin(image_subscriber)
   
#   # Destroy the node explicitly
#   # (optional - otherwise it will be done automatically
#   # when the garbage collector destroys the node object)
#   image_subscriber.destroy_node()
   
#   # Shutdown the ROS client library for Python
#   rclpy.shutdown()
   
# if __name__ == '__main__':
#   main()



import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from gui_test import GUI
import sys
from PyQt5.QtWidgets import QApplication

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image, 
            'video_frames', 
            self.listener_callback, 
            10)
        self.subscription

        self.br = CvBridge()
        self.current_frame = None

    def listener_callback(self, data):
        self.current_frame = self.br.imgmsg_to_cv2(data)

    def get_frame(self):
        return self.current_frame

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    image_subscriber = ImageSubscriber()
    gui = GUI(image_subscriber)
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
