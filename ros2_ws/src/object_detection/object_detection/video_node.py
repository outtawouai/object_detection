import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

VIDEO_PATH = "/home/object_detection/input/sidewalk.mp4"

class ImagePublisher(Node):

    def __init__(self):
        super().__init__('video_node')
        self.image_publisher = self.create_publisher(Image, 'topic', 10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.bridge = CvBridge()
        self.is_video_loaded = False

    def timer_callback(self):

        if not self.is_video_loaded :
            self.get_logger().info("Opening video in " + str(VIDEO_PATH))
            self.cap = cv2.VideoCapture(VIDEO_PATH)
            self.is_video_loaded = True
        
        if self.cap.isOpened() == False:
            self.get_logger().info("Could not open video in" + str(VIDEO_PATH))
            exit()

        ret, frame = self.cap.read()
        if ret :
            cv2.imshow('Frame', frame)
            cv2.waitKey(3)
            try:
                frame_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            except CvBridgeError as e:
                print(e)
            else:
                self.image_publisher.publish(frame_msg)
        else:
            self.is_video_loaded = False


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ImagePublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
