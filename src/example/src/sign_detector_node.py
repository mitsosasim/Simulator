#!/usr/bin/env python3
"""
YoloSignPoseNode

A ROS node that uses YOLO (via torch.hub) to detect traffic signs in a camera image,
estimates their 3D pose in the camera frame, and publishes both a PoseArray and
individual TF frames for each sign. Bounding boxes & labels are colored per-class
(like YOLO), while pose XYZ text remains green (fallback) or blue (PnP).
"""

import os
import sys
import rospy
import cv2
import numpy as np
import tf2_ros
import torch
import tf.transformations as tfm
import warnings
warnings.filterwarnings("ignore", category=FutureWarning)
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from std_msgs.msg import MultiArrayDimension
from utils_ros.msg import SignLabelArray

class YoloSignPoseNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('yolo_sign_pose_node')

        # —————————————— PARAMETERS ——————————————
        # Load parameters for model path, YOLO repo, confidence threshold, and device
        model_path = rospy.get_param(
            '~model',
            '/home/brakingbad/Documents/Simulator/YOLOv5-traffic/yolov5/'
            'runs/train/traffic_signs/weights/best.pt'
        )
        yolo_repo = '/home/brakingbad/Documents/Simulator/YOLOv5-traffic/yolov5'
        conf_th   = rospy.get_param('~conf', 0.6)
        device    = rospy.get_param('~device', 'cuda')

        # Default real-world sign dimensions (cm) for fallback
        self.sign_default_width  = 6.0
        self.sign_default_height = 6.0

        # Mapping from sign prefix to (width_cm, height_cm, shape)
        self.sign_specs = {
            'crosswalk':    (6.0, 6.0, 'rectangle'),
            'parking':      (6.0, 6.0, 'rectangle'),
            'oneway':       (6.0, 6.0, 'rectangle'),
            'enterhighway': (4.2, 6.0, 'rectangle'),
            'exithighway':  (4.2, 6.0, 'rectangle'),
            'noentry':      (6.0, 6.0, 'circle'),
            'roundabout':   (6.0, 6.0, 'triangle'),
            'priority':     (6.0, 6.0, 'diamond'),
        }

        # —————————————— LOAD YOLOv5 ——————————————
        # Ensure local yolov5 repo is first on PYTHONPATH for torch.hub
        if os.path.isdir(yolo_repo) and yolo_repo not in sys.path:
            sys.path.insert(0, yolo_repo)

        rospy.loginfo(f"[YOLO] Loading model from {model_path}")
        # Load YOLOv5 model using torch.hub
        self.model = torch.hub.load(
            yolo_repo, 'custom', path=model_path, source='local'
        )
        self.model.conf = conf_th
        # Move model to CUDA if available and requested
        if 'cuda' in device and torch.cuda.is_available():
            self.model.to(device)
        rospy.loginfo(f"[YOLO] CUDA available? {torch.cuda.is_available()}")

        # Log the actual class names for debugging
        rospy.loginfo(f"[YOLO] Class names: {self.model.names}")

        # —————————————— SET UP ROS ——————————————
        self.bridge = CvBridge()  # For converting ROS <-> OpenCV images
        self.tf_br  = tf2_ros.TransformBroadcaster()  # For publishing TF frames

        self.cam_K = None  # Camera intrinsic matrix
        self.dist  = None  # Camera distortion coefficients

        # Subscribe to camera info to get intrinsics
        rospy.Subscriber(
            '/automobile/camera_info',
            CameraInfo,
            self.info_cb,
            queue_size=1
        )
        # Subscribe to raw camera images
        rospy.Subscriber(
            '/automobile/image_raw',
            Image,
            self.image_cb,
            queue_size=1,
            buff_size=4*1024*1024
        )

        # Publishers for annotated image, detected poses, and sign labels
        self.pub_img    = rospy.Publisher(
            '/automobile/image_annotated', Image, queue_size=1
        )
        self.pub_poses  = rospy.Publisher(
            '/sign_detector/sign_poses', PoseArray, queue_size=1
        )
        self.pub_labels = rospy.Publisher(
            '/sign_detector/sign_labels', SignLabelArray, queue_size=1
        )

        # Assign a unique color per class (BGR), reproducibly using a fixed seed
        rng = np.random.RandomState(42)
        self.class_colors = {
            name: tuple(int(c) for c in rng.randint(0,256,3)[::-1])
            for name in self.model.names.values()
        }

        # Create OpenCV window for visualization
        cv2.namedWindow('Detections', cv2.WINDOW_NORMAL)
        self.last_annotated = None  # Store last annotated image for display
        rospy.on_shutdown(self.on_shutdown)  # Clean up on shutdown

    def on_shutdown(self):
        # Close OpenCV windows and log shutdown
        cv2.destroyAllWindows()
        rospy.loginfo("[YOLO] Closed OpenCV windows")

    def info_cb(self, msg: CameraInfo):
        # Callback to receive camera intrinsics and distortion coefficients
        if self.cam_K is None:
            self.cam_K = np.array(msg.K).reshape(3,3)
            self.dist  = np.array(msg.D)
            rospy.loginfo(
                f"[PoseNode] fx={self.cam_K[0,0]:.1f}, fy={self.cam_K[1,1]:.1f}, "
                f"cx={self.cam_K[0,2]:.1f}, cy={self.cam_K[1,2]:.1f}"
            )

    @staticmethod
    def order_pts(pts):
        # Orders four points as top-left, top-right, bottom-right, bottom-left
        xSorted = pts[np.argsort(pts[:,0]),:]
        left, right = xSorted[:2], xSorted[2:]
        tl, bl = left[np.argsort(left[:,1])]
        tr, br = right[np.argsort(right[:,1])]
        return np.array([tl, tr, br, bl], dtype=np.float32)

    def fallback_dist(self, cx, cy, pix_h, pix_w, real_w, real_h):
        # Fallback pinhole model for estimating 3D position from bounding box center and size
        Z_h = (self.cam_K[1,1] * real_h) / pix_h
        Z_w = (self.cam_K[0,0] * real_w) / pix_w
        Z   = 0.5 * (Z_h + Z_w)
        X   = (cx - self.cam_K[0,2]) * Z / self.cam_K[0,0]
        Y   = (cy - self.cam_K[1,2]) * Z / self.cam_K[1,1]
        return np.array([X, Y, Z], dtype=np.float32)

    def image_cb(self, msg: Image):
        # Main callback for each incoming camera image
        if self.cam_K is None:
            return  # Wait for camera intrinsics

        # Convert ROS image to OpenCV and undistort
        frame_bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        frame_u   = cv2.undistort(frame_bgr, self.cam_K, self.dist, None, self.cam_K)

        # Convert to RGB for YOLO and run inference
        frame_rgb = cv2.cvtColor(frame_u, cv2.COLOR_BGR2RGB)
        results   = self.model(frame_rgb)
        dets      = results.xyxy[0].cpu().numpy()  # x1,y1,x2,y2,conf,cls

        annotated = frame_u.copy()  # Copy for annotation
        poses     = PoseArray(header=msg.header)  # Prepare PoseArray for output

        for i, (x1, y1, x2, y2, conf, cls) in enumerate(dets):
            # Parse detection bounding box and class
            x1, y1, x2, y2 = map(int, (x1,y1,x2,y2))
            w, h          = x2 - x1, y2 - y1
            cx, cy        = x1 + w/2, y1 + h/2

            cls = int(cls)
            cls_name = self.model.names[cls]

            # Find matching sign spec by prefix, or use default
            match = next(
                (k for k in self.sign_specs if cls_name.startswith(k)),
                None
            )
            if match:
                real_w, real_h, shape = self.sign_specs[match]
            else:
                real_w, real_h, shape = (
                    self.sign_default_width,
                    self.sign_default_height,
                    'rectangle'
                )

            # Extract ROI and find contours for pose estimation
            roi        = cv2.cvtColor(frame_u[y1:y1+h, x1:x1+w],
                                      cv2.COLOR_BGR2GRAY)
            blur       = cv2.GaussianBlur(roi, (5,5), 0)
            edges      = cv2.Canny(blur, 50, 150)
            cnts, _    = cv2.findContours(edges, cv2.RETR_EXTERNAL,
                                          cv2.CHAIN_APPROX_SIMPLE)
            if not cnts:
                continue
            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) < 0.1*w*h:
                continue

            # Approximate contour to polygon and extract points
            approx  = cv2.approxPolyDP(c, 0.02*cv2.arcLength(c, True), True)
            pts     = approx.reshape(-1,2).astype(np.float32) + np.array([x1, y1])
            rvec, tvec, used_pnp = None, None, False

            # Try to estimate pose using PnP for known shapes
            if shape=='rectangle' and len(pts)==4:
                img_pts = self.order_pts(pts)
                obj_pts = np.array([
                    [0,0,0], [real_w,0,0],
                    [real_w,real_h,0], [0,real_h,0]
                ], dtype=np.float32)
                ok, rvec, tvec, _ = cv2.solvePnPRansac(
                    obj_pts, img_pts, self.cam_K, self.dist,
                    reprojectionError=8.0, iterationsCount=100,
                    confidence=0.99, flags=cv2.SOLVEPNP_ITERATIVE
                )
                used_pnp = bool(ok)

            elif shape=='triangle' and len(pts)==3:
                obj_pts = np.array([
                    [0,0,0], [real_w,0,0],
                    [real_w/2, real_h, 0]
                ], dtype=np.float32)
                ok, rvec, tvec = cv2.solvePnP(
                    obj_pts, pts, self.cam_K, self.dist
                )
                used_pnp = bool(ok)

            elif shape=='diamond' and len(pts)==4:
                img_pts = self.order_pts(pts)
                w2, h2  = real_w/2, real_h/2
                obj_pts = np.array([
                    [0,-h2,0], [w2,0,0],
                    [0,h2,0], [-w2,0,0]
                ], dtype=np.float32)
                ok, rvec, tvec = cv2.solvePnP(
                    obj_pts, img_pts, self.cam_K, self.dist
                )
                used_pnp = bool(ok)

            elif shape=='circle':
                # For circles, estimate pose using bounding circle and pinhole model
                (cx_img, cy_img), radius = cv2.minEnclosingCircle(c)
                Z = (self.cam_K[1,1] * real_h) / (2*radius)
                X = (cx_img - self.cam_K[0,2]) * Z / self.cam_K[0,0]
                Y = (cy_img - self.cam_K[1,2]) * Z / self.cam_K[1,1]
                tvec = np.array([[X],[Y],[Z]], dtype=np.float32)

            # If PnP failed, use fallback pinhole model
            if tvec is None:
                fallback = self.fallback_dist(cx, cy, h, w, real_w, real_h)
                tvec     = fallback.reshape(3,1)
                used_pnp = False

            # — draw class-colored box & label —
            box_color  = self.class_colors[cls_name]
            cv2.rectangle(annotated, (x1,y1), (x2,y2), box_color, 2)

            label = f"{cls_name} {conf:.2f}"
            (tw, th), baseline = cv2.getTextSize(label,
                                                 cv2.FONT_HERSHEY_SIMPLEX,
                                                 0.5, 1)
            cv2.rectangle(
                annotated,
                (x1, y1 - th - baseline - 4),
                (x1 + tw + 4, y1),
                box_color,
                cv2.FILLED
            )
            cv2.putText(
                annotated, label,
                (x1 + 2, y1 - baseline - 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (255,255,255), 1, cv2.LINE_AA
            )

            # — annotate 3D pose in blue (PnP) or green (fallback) —
            text_col = (255,0,0) if used_pnp else (0,255,0)
            x_m, y_m, z_m = (tvec.flatten() / 100.0)  # Convert cm to meters
            cv2.putText(annotated, f"X={x_m:.2f}m",
                        (x1, y1 - th - baseline - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_col, 2)
            cv2.putText(annotated, f"Y={y_m:.2f}m",
                        (x1, y1 - th - baseline - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_col, 2)
            cv2.putText(annotated, f"Z={z_m:.2f}m",
                        (x1, y1 - th - baseline - 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_col, 2)

            # — build & publish Pose + TF —
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x_m, y_m, z_m
            if rvec is not None:
                # Convert rotation vector to quaternion for orientation
                R, _ = cv2.Rodrigues(rvec)
                M    = np.eye(4); M[:3,:3] = R
                q    = tfm.quaternion_from_matrix(M)
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
            else:
                pose.orientation.w = 1.0  # Default orientation if not available
            poses.poses.append(pose)

            # Publish a TF frame for each detected sign
            tf_msg = TransformStamped()
            tf_msg.header         = msg.header
            tf_msg.child_frame_id = f"sign_{i}"
            tf_msg.transform.translation.x = x_m
            tf_msg.transform.translation.y = y_m
            tf_msg.transform.translation.z = z_m
            tf_msg.transform.rotation       = pose.orientation
            self.tf_br.sendTransform(tf_msg)

        # — publish annotated image, poses & labels —
        self.last_annotated = annotated
        out_img = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        out_img.header = msg.header
        self.pub_img.publish(out_img)
        self.pub_poses.publish(poses)

        # Publish sign labels as a custom message
        labels_msg = SignLabelArray()
        labels_msg.labels = [
            self.model.names[int(row[5])].split('_')[0]
            for row in dets
        ]
        self.pub_labels.publish(labels_msg)

    def spin(self):
        # Main loop for displaying annotated images in a window
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.last_annotated is not None:
                cv2.imshow('Detections', self.last_annotated)
                cv2.waitKey(1)
            rate.sleep()


if __name__ == '__main__':
    # Instantiate and run the node
    node = YoloSignPoseNode()
    node.spin()