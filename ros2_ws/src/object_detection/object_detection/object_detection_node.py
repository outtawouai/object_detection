import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from custom_messages.msg import AllDetections, Detection, Box
import numpy as np

class YoloObjectDetector(Node):

    def __init__(self):
        super().__init__('object_detector_node')
        self.image_subscriber = self.create_subscription(
            Image,
            'images',
            self.image_callback,
            10
        )
        self.detections_publisher = self.create_publisher(
            AllDetections, 'object_detections', 10
        )
        self.bridge = CvBridge()

        self.model = YOLO("yolo11s.pt")

    def image_callback(self, msg):
        image_cv2 = self.bridge.imgmsg_to_cv2(msg)

        yolo_result = self.model(image_cv2, verbose=False)
        header = msg.header

        message = self.getMsgFromYoloResult(yolo_result, header)

        self.detections_publisher.publish(message)

    def getMsgFromYoloResult(self, yolo_result, header):
        detections = []
        result = yolo_result[0] #batch size is 0 since only one image

        class_names_dict = result.names

        id=0
        for box in result.boxes :
            cls = int(box.cls)
            cls_type = class_names_dict[cls]
            conf = float(box.conf)
            box_xyxy = box.xyxy.numpy().astype(np.float64)[0]

            new_detection = Detection()
            new_box = Box()
            new_box.xyxy = box_xyxy
            
            new_detection.cls = cls
            new_detection.cls_type = cls_type
            new_detection.detect_id = id
            new_detection.confidence = conf
            new_detection.box = new_box
            
            detections.append(new_detection)

            id+=1

        toreturn_msg = AllDetections()
        toreturn_msg.detections = detections
        toreturn_msg.header = header

        self.get_logger().info(str(toreturn_msg))

        return toreturn_msg


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = YoloObjectDetector()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
