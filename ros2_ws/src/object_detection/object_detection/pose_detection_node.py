import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from custom_messages.msg import AllPoseDetections, Pose, Keypoint, Box
import numpy as np

class YoloPoseDetector(Node):

    def __init__(self):
        super().__init__('pose_detector_node')
        self.image_subscriber = self.create_subscription(
            Image,
            'images',
            self.image_callback,
            10
        )
        self.pose_detections_publisher = self.create_publisher(
            AllPoseDetections, 'pose_detections', 10
        )
        self.bridge = CvBridge()

        self.model = YOLO("yolo11s-pose.pt")

    def image_callback(self, msg):
        image_cv2 = self.bridge.imgmsg_to_cv2(msg)

        yolo_pose_result = self.model(image_cv2, verbose=False)
        header = msg.header

        message = self.getMsgFromYoloPoseResult(yolo_pose_result, header)

        self.pose_detections_publisher.publish(message)

    def getMsgFromYoloPoseResult(self, yolo_result, header):

        detections = []
        result = yolo_result[0] #batch size is 0 since only one image

        class_names_dict = result.names

        id=0
        for (box, keypoints) in zip(result.boxes, result.keypoints) :
            cls = int(box.cls)
            cls_type = class_names_dict[cls]
            conf = float(box.conf)
            box_xyxy = box.xyxy.numpy().astype(np.float64)[0]

            new_pose = Pose()
            new_box = Box()
            new_box.xyxy = box_xyxy
            
            new_pose.cls = cls
            new_pose.cls_type = cls_type
            new_pose.detect_id = id
            new_pose.confidence = conf
            new_pose.box = new_box
            
            keypoints_data = keypoints.data[0]

            keypoints_msgs = [] 
            for data in keypoints_data:
                # nested for loop... find a way to vectorize this next time, this looks slow
                keypoint_msg = Keypoint()
                keypoint_msg.x = float(data[0])
                keypoint_msg.y = float(data[1])
                keypoint_msg.confidence = float(data[2])

                keypoints_msgs.append(keypoint_msg)

            new_pose.keypoints = keypoints_msgs

            detections.append(new_pose)

            id+=1

        toreturn_msg = AllPoseDetections()
        toreturn_msg.detections = detections
        toreturn_msg.header = header

        return toreturn_msg


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = YoloPoseDetector()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
