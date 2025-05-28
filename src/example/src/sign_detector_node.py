#!/usr/bin/env python3
"""
YoloSignPoseNode

A ROS node that uses YOLO to detect traffic signs in a camera image,
estimates their 3D pose in the camera frame, and publishes both a
PoseArray and individual TF frames for each sign.

Enhancements:
- Handles multiple sign shapes (rectangles, circles, triangles, diamonds)
  by mapping each YOLO class to its real-world dimensions and shape.
- Uses RANSAC-backed solvePnP for robust rectangle pose estimation.
- Preprocesses contours with Gaussian blur and Canny edge detection.
- Falls back to a combined pinhole estimate using both real height and width
  when PnP fails, ensuring correct aspect ratio for highway signs.
"""

#!/usr/bin/env python3
import os, sys

# 1) Prepend the YOLOv5 repo root so its "utils" is found first:
yolo_repo = '/home/brakingbad/Documents/Simulator/YOLOv5-traffic/yolov5'
if os.path.isdir(yolo_repo) and yolo_repo not in sys.path:
    sys.path.insert(0, yolo_repo)

# 2) Optionally, remove your ROS utils if already on sys.path:
#    (uncomment if needed)
# 
# ros_utils = os.path.join(os.getenv('HOME'),
#                          'Documents/Simulator/src/utils')
# if ros_utils in sys.path:
#     sys.path.remove(ros_utils)
# # 


import rospy
import cv2
import numpy as np
import tf2_ros
import torch

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from ultralytics import YOLO
from std_msgs.msg import MultiArrayDimension
from utils.msg import SignLabelArray  



import tf.transformations as tfm


class YoloSignPoseNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('yolo_sign_pose_node')

        # Load parameters
        model_path = rospy.get_param('~model',  '/home/brakingbad/Documents/Simulator/YOLOv5-traffic/yolov5/runs/train/traffic_signs/weights/best.pt')
        repo_dir = '/home/brakingbad/Documents/Simulator/YOLOv5-traffic/yolov5'
        conf       = rospy.get_param('~conf',   0.6)
        device     = rospy.get_param('~device','cuda')

        # Default cm values in case a class is missing
        self.sign_default_height = 6.0
        self.sign_default_width  = 6.0

        # Mapping from class prefix to (width_cm, height_cm, shape)
        self.sign_specs = {
            'crosswalk':   (6.0, 6.0, 'rectangle'),
            'parking':     (6.0, 6.0, 'rectangle'),
            'oneway':      (6.0, 6.0, 'rectangle'),
            'enterhighway':(4.2, 6.0, 'rectangle'),
            'exithighway': (4.2, 6.0, 'rectangle'),
            'noentry':     (6.0, 6.0, 'circle'),
            'roundabout':  (6.0, 6.0, 'triangle'),
            'priority':    (6.0, 6.0, 'diamond'),
        }


        rospy.loginfo(f"[YOLO] CUDA available? {torch.cuda.is_available()}")

        #Load the YOLO model
        self.model = torch.hub.load(
            yolo_repo,  # repo dir
            'custom',          # custom model
            path=model_path,   # your .pt
            source='local'     # local repo
        )

        # self.model = torch.hub.load(
        #     'ultralytics/yolov5',  # repo dir
        #     'custom',          # custom model
        #     path=model_path,   # your .pt
        #     force_reload=True     
        # )

        self.model.conf = conf
        if 'cuda' in device and torch.cuda.is_available():
            self.model.to(device)

        # CV Bridge and TF broadcaster
        self.bridge = CvBridge()
        self.tf_br  = tf2_ros.TransformBroadcaster()

        # Camera intrinsics placeholders
        self.cam_K = None
        self.dist  = None
        rospy.Subscriber('/automobile/camera_info',
                         CameraInfo, self.info_cb, queue_size=1)

        # Image subscriber and publishers
        self.sub_img   = rospy.Subscriber('/automobile/image_raw',
                                          Image, self.image_cb,
                                          queue_size=1, buff_size=4*1024*1024)
        self.pub_img   = rospy.Publisher('/automobile/image_annotated',
                                         Image, queue_size=1)
        self.pub_poses = rospy.Publisher('/sign_detector/sign_poses',
                                         PoseArray, queue_size=1)
        self.pub_labels = rospy.Publisher('/sign_detector/sign_labels',
                                          SignLabelArray, queue_size=1)
        

        # OpenCV window for visualization
        cv2.namedWindow('Detections', cv2.WINDOW_NORMAL)
        self.last_annotated = None

        rospy.on_shutdown(self.on_shutdown)

    def on_shutdown(self):
        cv2.destroyAllWindows()
        rospy.loginfo("[YOLO] Closed OpenCV windows")

    def info_cb(self, msg: CameraInfo):
        """
        CameraInfo callback to read intrinsic matrix K and distortion D.
        """
        if self.cam_K is None:
            self.cam_K = np.array(msg.K).reshape(3,3)
            self.dist  = np.array(msg.D)
            rospy.loginfo(f"[PoseNode] fx={self.cam_K[0,0]:.1f}, "
                          f"fy={self.cam_K[1,1]:.1f}, cx={self.cam_K[0,2]:.1f}, "
                          f"cy={self.cam_K[1,2]:.1f}")

    @staticmethod
    def order_pts(pts):
        """
        Orders 4 corner points: top-left, top-right, bottom-right, bottom-left.
        """
        xSorted = pts[np.argsort(pts[:,0]),:]
        left, right = xSorted[:2], xSorted[2:]
        tl, bl = left[np.argsort(left[:,1])]
        tr, br = right[np.argsort(right[:,1])]
        return np.array([tl, tr, br, bl], dtype=np.float32)

    def fallback_dist(self, cx, cy, pix_h, pix_w, real_w, real_h):
        """
        Combined pinhole fallback:
        - Compute Z from both real height/pixel height and real width/pixel width.
        - Average the two estimates for robust scaling when aspect ratio != 1.
        """
        # Z estimate from height
        Z_h = (self.cam_K[1,1] * real_h) / pix_h
        # Z estimate from width
        Z_w = (self.cam_K[0,0] * real_w) / pix_w
        # Combined depth (cm)
        Z = 0.5 * (Z_h + Z_w)
        # Reproject to X, Y
        X = (cx - self.cam_K[0,2]) * Z / self.cam_K[0,0]
        Y = (cy - self.cam_K[1,2]) * Z / self.cam_K[1,1]
        return np.array([X, Y, Z], dtype=np.float32)

    def image_cb(self, msg: Image):
        """
        Image callback: runs YOLO, extracts contours, fits PnP or fallback,
        and publishes PoseArray + TFs.
        """
        if self.cam_K is None:
            return  # wait for intrinsics

        # Convert and undistort
        frame   = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        frame_u = cv2.undistort(frame, self.cam_K, self.dist, None, self.cam_K)

        # YOLO inference
        #results = self.model(frame_u)
        

        #results.render()                    # draw boxes on results.imgs
        #annotated = results.imgs[0].copy()  # get the annotated frame
        results = self.model(frame_u)
        annotated = frame_u.copy()
        

        poses = PoseArray(header=msg.header)
        det = results.xyxy[0].cpu().numpy()  # shape (N,6): x1,y1,x2,y2,conf,cls
       # Process each detection
        for i, row in enumerate(det):
            # unpack all six at once
            x1f, y1f, x2f, y2f, conf, cls = row
            # convert box corners to ints
            x1, y1, x2, y2 = map(int, (x1f, y1f, x2f, y2f))
            w, h = x2 - x1, y2 - y1
            cx, cy = x1 + w/2, y1 + h/2

            # Look up sign specs
            cls_name = self.model.names[int(cls)]
            key = cls_name.split('_')[0]
            real_w, real_h, shape = self.sign_specs.get(
                key,
                (self.sign_default_width,
                 self.sign_default_height,
                 'rectangle')
            )

            # Preprocess ROI for contour detection
            roi_gray = cv2.cvtColor(frame_u[y1:y1+h, x1:x1+w],
                                    cv2.COLOR_BGR2GRAY)
            blur  = cv2.GaussianBlur(roi_gray, (5,5), 0)
            edges = cv2.Canny(blur, 50, 150)

            cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
            if not cnts:
                continue
            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) < 0.1*w*h:
                continue

            # Initialize pose variables
            rvec = None
            tvec = None
            used_pnp = False

            # Approximate polygon
            approx = cv2.approxPolyDP(c, 0.02*cv2.arcLength(c, True), True)
            pts = (approx.reshape(-1,2).astype(np.float32)
                   + np.array([x1, y1]))

            # Shape-specific estimation
            if shape == 'rectangle' and len(pts) == 4:
                img_pts = self.order_pts(pts)
                obj_pts = np.array([[0, 0, 0],
                                    [real_w, 0, 0],
                                    [real_w, real_h, 0],
                                    [0, real_h, 0]],
                                   dtype=np.float32)
                ok, rvec, tvec, _ = cv2.solvePnPRansac(
                    obj_pts, img_pts, self.cam_K, self.dist,
                    reprojectionError=8.0, iterationsCount=100,
                    confidence=0.99, flags=cv2.SOLVEPNP_ITERATIVE)
                if ok:
                    used_pnp = True
                else:
                    rvec, tvec = None, None

            elif shape == 'triangle' and len(pts) == 3:
                img_pts = pts
                obj_pts = np.array([[0, 0, 0],
                                    [real_w, 0, 0],
                                    [real_w/2, real_h, 0]],
                                   dtype=np.float32)
                ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts,
                                              self.cam_K, self.dist)
                if ok:
                    used_pnp = True
                else:
                    rvec, tvec = None, None

            elif shape == 'diamond' and len(pts) == 4:
                img_pts = self.order_pts(pts)
                w2, h2 = real_w/2, real_h/2
                obj_pts = np.array([[ 0, -h2, 0],
                                    [ w2,  0, 0],
                                    [ 0,  h2, 0],
                                    [-w2,  0, 0]],
                                   dtype=np.float32)
                ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts,
                                              self.cam_K, self.dist)
                if not ok:
                    rvec, tvec = None, None

            elif shape == 'circle':
                (cx_img, cy_img), radius = cv2.minEnclosingCircle(c)
                Z = (self.cam_K[1,1]*real_h) / (2*radius)
                X = (cx_img - self.cam_K[0,2]) * Z / self.cam_K[0,0]
                Y = (cy_img - self.cam_K[1,2]) * Z / self.cam_K[1,1]
                tvec = np.array([[X], [Y], [Z]], dtype=np.float32)
                used_pnp = False # Fallback to pinhole estimate

            # If PnP failed, use combined pinhole fallback
            if tvec is None:
                fallback = self.fallback_dist(cx, cy, h, w, real_w, real_h)
                tvec = fallback.reshape(3,1)
                used_pnp = False

            # Draw bounding box
            # cv2.rectangle(annotated, (x1, y1), (x2, y2),
            #               (0, 255, 0)if not used_pnp else (255, 0, 0), 2)

            # Draw bounding box
            color = (255, 0, 0) if used_pnp else (0, 255, 0)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

            # Prepare label with class name and confidence
            label = f"{cls_name} {conf:.2f}"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0
            thickness = 1

            # Measure text size to draw background
            (text_w, text_h), baseline = cv2.getTextSize(label, font, font_scale, thickness)
            # Draw filled rectangle for label background
            cv2.rectangle(
                annotated,
                (x1, y1 - text_h - baseline - 4),
                (x1 + text_w, y1),
                color,
                cv2.FILLED
            )
            # Draw label text in white
            cv2.putText(
                annotated,
                label,
                (x1, y1 - baseline - 2),
                font,
                font_scale,
                (255, 255, 255),
                thickness,
                lineType=cv2.LINE_AA
            )

            # Convert to meters
            x_m, y_m, z_m = (tvec.flatten() / 100.0)

            # Annotate 3D coordinates
            text_color = (255, 0, 0) if used_pnp else (0, 255, 0)  # match box color
            cv2.putText(annotated, f"X={x_m:.2f}m", (x1, y1 - text_h - baseline - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 2)
            cv2.putText(annotated, f"Y={y_m:.2f}m", (x1, y1 - text_h - baseline - 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 2)
            cv2.putText(annotated, f"Z={z_m:.2f}m", (x1, y1 - text_h - baseline - 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 2)


            # Build ROS Pose
            p = Pose()
            p.position.x = x_m
            p.position.y = y_m
            p.position.z = z_m

            if rvec is not None:
                R, _ = cv2.Rodrigues(rvec)
                M = np.eye(4); M[:3,:3] = R
                q = tfm.quaternion_from_matrix(M)
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
            else:
                p.orientation.w = 1.0

            poses.poses.append(p)




            # Broadcast a TF frame
            tf = TransformStamped()
            tf.header         = msg.header
            tf.child_frame_id = f"sign_{i}"
            tf.transform.translation.x = x_m
            tf.transform.translation.y = y_m
            tf.transform.translation.z = z_m
            tf.transform.rotation       = p.orientation
            self.tf_br.sendTransform(tf)

        # Store for visualization
        self.last_annotated = annotated.copy()

        # Publish annotated image and poses
        out_img = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        out_img.header = msg.header
        self.pub_img.publish(out_img)
        self.pub_poses.publish(poses)

        labels_msg = SignLabelArray()
        labels_msg.labels = []  # ensure fresh list
        for row in det:
            cls = int(row[5])
            # use same generic key as EKF expects
            key = self.model.names[cls].split('_')[0]
            labels_msg.labels.append(key)
        self.pub_labels.publish(labels_msg)


if __name__ == '__main__':
    node = YoloSignPoseNode()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if node.last_annotated is not None:
            cv2.imshow('Detections', node.last_annotated)
            cv2.waitKey(1)
        rate.sleep()
