import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from custom_messages.msg import AllDetections, Detection, Box, AllPoseDetections, Pose, Keypoint
import numpy as np
import cv2
from cv_bridge import CvBridge
import message_filters
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class YoloObjectDetector(Node):

    def __init__(self):
        super().__init__('object_detector_node')

        self.bones_to_display = {
            "1" : [2, 3],
            "2" : [3, 4],
            "3" : [5],
            "6" : [7, 8, 12],
            "7" : [9, 13],
            "8" : [10],
            "9" : [11],
            "12" : [14, 13],
            "13" : [15],
            "14" : [16],
            "15" : [17]
        }

        profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.image_subscriber = message_filters.Subscriber(
            self,
            Image,
            "images",
            qos_profile = profile
        )
        self.obj_detection_subscriber = message_filters.Subscriber(
            self,
            AllDetections,
            "object_detections",
            qos_profile = profile
        )
        self.pose_detection_subscriber = message_filters.Subscriber(
            self,
            AllPoseDetections,
            "pose_detections",
            qos_profile = profile
        )

        subs = [self.image_subscriber, self.obj_detection_subscriber, self.pose_detection_subscriber]
        self.ts = message_filters.TimeSynchronizer(subs, 10)
        self.ts.registerCallback(self.callback)

        self.bridge = CvBridge()

    def callback(self, frame_msg:Image, detector_msg:AllDetections, pose_msg:AllPoseDetections):
        image_cv2 = self.bridge.imgmsg_to_cv2(frame_msg)

        for detection in detector_msg.detections:

            if detection.confidence < 0.5:
                continue

            topleft_corner = (int(detection.box.xyxy[0]), int(detection.box.xyxy[1]))
            botright_corner = (int(detection.box.xyxy[2]), int(detection.box.xyxy[3]))

            if detection.cls == 0:
                color = (0, 0, 255)
            else:
                color = (255, 0, 0)

            image_cv2 = cv2.rectangle(image_cv2, topleft_corner, botright_corner, color, 3)
            cv2.putText(
                image_cv2,
                detection.cls_type,
                (int(detection.box.xyxy[0] + 10), int(detection.box.xyxy[1]) + 15),
                fontFace = cv2.FONT_HERSHEY_SIMPLEX,
                fontScale = 0.6,
                color = color,
                thickness = 2
            )

        for pose in pose_msg.detections:
            i = 1
            for keypoint in pose.keypoints:
                if keypoint.confidence > 0.5:
                    image_cv2 = cv2.circle(image_cv2, (int(keypoint.x), int(keypoint.y)), 2, (0, 0, 255), 2)
                    if self.bones_to_display.get(str(i)) is not None:
                        for second_kp in self.bones_to_display.get(str(i)):
                            if pose.keypoints[second_kp - 1].confidence > 0.5 :
                                point_a = (int(keypoint.x), int(keypoint.y))
                                point_b = (int(pose.keypoints[second_kp - 1].x), int(pose.keypoints[second_kp - 1].y))
                                image_cv2 = cv2.line(image_cv2, point_a, point_b, (0, 0, 255), 2)
                i+=1


        image_cv2 = cv2.resize(image_cv2, (1280, 720)) 
        cv2.imshow("Object detection", image_cv2)
        cv2.waitKey(3)



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

