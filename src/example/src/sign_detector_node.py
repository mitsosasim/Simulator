#!/usr/bin/env python3
"""
YoloSignPoseNode

A ROS node that uses YOLO (via torch.hub) to detect traffic signs in a camera image,
estimates their 3D pose in the camera frame, converts to ROS camera-optical coords,
transforms into chassis::link, and publishes both a PoseArray and sign labels.
Also publishes annotated image and TF frames for each sign (once per timestamp).
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
import tf2_geometry_msgs
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray, TransformStamped, PoseStamped
from utils_ros.msg import SignLabelArray

warnings.filterwarnings("ignore", category=FutureWarning)

class YoloSignPoseNode:
    def __init__(self):
        rospy.init_node('yolo_sign_pose_node')

        # PARAMETERS
        model_path = rospy.get_param('~model',
            '/home/brakingbad/Documents/Simulator/YOLOv5-traffic/yolov5/'
            'runs/train/traffic_signs/weights/best.pt'
        )
        yolo_repo = rospy.get_param('~yolo_repo',
            '/home/brakingbad/Documents/Simulator/YOLOv5-traffic/yolov5'
        )
        conf_th = rospy.get_param('~conf', 0.6)
        device  = rospy.get_param('~device', 'cuda')

        # Default sign dims (cm)
        self.sign_default_width  = 6.0
        self.sign_default_height = 6.0
        # Mapping from prefix to (width_cm, height_cm, shape)
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

        # Load YOLOv5 model
        if os.path.isdir(yolo_repo) and yolo_repo not in sys.path:
            sys.path.insert(0, yolo_repo)
        rospy.loginfo(f"[YOLO] Loading model from {model_path}")
        self.model = torch.hub.load(yolo_repo, 'custom', path=model_path, source='local')
        self.model.conf = conf_th
        if 'cuda' in device and torch.cuda.is_available():
            self.model.to(device)
        rospy.loginfo(f"[YOLO] CUDA available? {torch.cuda.is_available()}")
        rospy.loginfo(f"[YOLO] Class names: {self.model.names}")

        # ROS setup
        self.bridge = CvBridge()
        self.tf_br = tf2_ros.TransformBroadcaster()

        self.cam_K = None
        self.dist  = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # To suppress duplicate TF per sign per timestamp
        self.last_tf_stamps = {}

        rospy.Subscriber('/automobile/camera_info', CameraInfo, self.info_cb, queue_size=1)
        rospy.Subscriber('/automobile/image_raw', Image, self.image_cb,
                         queue_size=1, buff_size=4*1024*1024)

        self.pub_img    = rospy.Publisher('/automobile/image_annotated', Image, queue_size=1)
        self.pub_poses  = rospy.Publisher('/sign_detector/sign_poses', PoseArray, queue_size=1)
        self.pub_labels = rospy.Publisher('/sign_detector/sign_labels', SignLabelArray, queue_size=1)

        # Colors per class
        rng = np.random.RandomState(42)
        self.class_colors = {
            name: tuple(int(c) for c in rng.randint(0,256,3)[::-1])
            for name in self.model.names.values()
        }

        cv2.namedWindow('Detections', cv2.WINDOW_NORMAL)
        self.last_annotated = None

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("[YOLO] Node initialized.")

    def on_shutdown(self):
        cv2.destroyAllWindows()
        rospy.loginfo("[YOLO] Closed OpenCV windows.")

    def info_cb(self, msg: CameraInfo):
        if self.cam_K is None:
            self.cam_K = np.array(msg.K).reshape(3,3)
            self.dist  = np.array(msg.D)
            rospy.loginfo(f"[YOLO] Camera intrinsics: fx={self.cam_K[0,0]:.1f}, fy={self.cam_K[1,1]:.1f}, cx={self.cam_K[0,2]:.1f}, cy={self.cam_K[1,2]:.1f}")

    @staticmethod
    def order_pts(pts):
        # Order 4 points: tl, tr, br, bl
        xSorted = pts[np.argsort(pts[:,0]),:]
        left, right = xSorted[:2], xSorted[2:]
        tl, bl = left[np.argsort(left[:,1])]
        tr, br = right[np.argsort(right[:,1])]
        return np.array([tl, tr, br, bl], dtype=np.float32)

    def fallback_dist(self, cx, cy, pix_h, pix_w, real_w, real_h):
        # Simple pinhole fallback: returns (X, Y, Z) in camera coords (cm)
        Z_h = (self.cam_K[1,1] * real_h) / pix_h
        Z_w = (self.cam_K[0,0] * real_w) / pix_w
        Z   = 0.5 * (Z_h + Z_w)
        X   = (cx - self.cam_K[0,2]) * Z / self.cam_K[0,0]
        Y   = (cy - self.cam_K[1,2]) * Z / self.cam_K[1,1]
        return np.array([X, Y, Z], dtype=np.float32)

    def image_cb(self, msg: Image):
        if self.cam_K is None:
            return  # wait for intrinsics

        # Convert and undistort
        frame_bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        frame_u = cv2.undistort(frame_bgr, self.cam_K, self.dist, None, self.cam_K)
        # YOLO expects RGB
        frame_rgb = cv2.cvtColor(frame_u, cv2.COLOR_BGR2RGB)
        results = self.model(frame_rgb)
        dets = results.xyxy[0].cpu().numpy()  # x1,y1,x2,y2,conf,cls

        annotated = frame_u.copy()
        poses = PoseArray(header=msg.header)
        labels_list = []

        for i, det in enumerate(dets):
            x1, y1, x2, y2, conf, cls_idx = det
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            w, h = x2 - x1, y2 - y1
            if w <= 0 or h <= 0:
                continue
            cx, cy = x1 + w/2.0, y1 + h/2.0
            cls_idx = int(cls_idx)
            cls_name = self.model.names[cls_idx]

            # Determine sign spec
            match = next((k for k in self.sign_specs if cls_name.startswith(k)), None)
            if match:
                real_w, real_h, shape = self.sign_specs[match]
            else:
                real_w, real_h, shape = (self.sign_default_width,
                                         self.sign_default_height,
                                         'rectangle')
            # ROI for contour
            roi = cv2.cvtColor(frame_u[y1:y1+h, x1:x1+w], cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(roi, (5,5), 0)
            edges = cv2.Canny(blur, 50, 150)
            cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cnts:
                continue
            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) < 0.1 * w * h:
                continue
            approx = cv2.approxPolyDP(c, 0.02 * cv2.arcLength(c, True), True)
            pts = approx.reshape(-1,2).astype(np.float32) + np.array([x1, y1], dtype=np.float32)
            rvec = None; tvec = None; used_pnp = False

            # PnP for known shapes
            if shape == 'rectangle' and pts.shape[0] == 4:
                img_pts = self.order_pts(pts)
                obj_pts = np.array([[0,0,0], [real_w,0,0],
                                    [real_w,real_h,0], [0,real_h,0]],
                                   dtype=np.float32)
                try:
                    ok, rvec_temp, tvec_temp, _ = cv2.solvePnPRansac(
                        obj_pts, img_pts, self.cam_K, self.dist,
                        reprojectionError=8.0, iterationsCount=100,
                        confidence=0.99, flags=cv2.SOLVEPNP_ITERATIVE
                    )
                    if ok:
                        rvec, tvec = rvec_temp, tvec_temp
                        used_pnp = True
                        # annotate corners
                        for j, pt in enumerate(img_pts.astype(int)):
                            cv2.circle(annotated, tuple(pt), 4, (0,255,255), -1)
                            cv2.putText(annotated, str(j),
                                        tuple(pt + 8),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                        (0,255,255), 2)
                        for j in range(4):
                            pt1 = tuple(img_pts[j].astype(int))
                            pt2 = tuple(img_pts[(j+1)%4].astype(int))
                            cv2.line(annotated, pt1, pt2, (0,255,255), 2)
                except cv2.error as e:
                    rospy.logwarn(f"[YOLO] PnP rectangle failed: {e}")
                    rvec, tvec, used_pnp = None, None, False

            elif shape == 'triangle' and pts.shape[0] == 3:
                obj_pts = np.array([[0,0,0], [real_w,0,0], [real_w/2, real_h, 0]], dtype=np.float32)
                try:
                    ok, rvec_temp, tvec_temp = cv2.solvePnP(
                        obj_pts, pts, self.cam_K, self.dist,
                        flags=cv2.SOLVEPNP_ITERATIVE
                    )
                    if ok:
                        rvec, tvec = rvec_temp, tvec_temp
                        used_pnp = True
                except cv2.error as e:
                    rospy.logwarn(f"[YOLO] PnP triangle failed: {e}")
                    rvec, tvec, used_pnp = None, None, False

            elif shape == 'diamond' and pts.shape[0] == 4:
                img_pts = self.order_pts(pts)
                w2, h2 = real_w/2.0, real_h/2.0
                obj_pts = np.array([[0,-h2,0], [w2,0,0], [0,h2,0], [-w2,0,0]], dtype=np.float32)
                try:
                    ok, rvec_temp, tvec_temp = cv2.solvePnP(obj_pts, img_pts, self.cam_K, self.dist)
                    if ok:
                        rvec, tvec = rvec_temp, tvec_temp
                        used_pnp = True
                except cv2.error as e:
                    rospy.logwarn(f"[YOLO] PnP diamond failed: {e}")
                    rvec, tvec, used_pnp = None, None, False

            elif shape == 'circle':
                # fallback circle: estimate Z from radius
                (cx_img, cy_img), radius = cv2.minEnclosingCircle(c)
                if radius > 1e-3:
                    Z = (self.cam_K[1,1] * real_h) / (2*radius)
                    X = (cx_img - self.cam_K[0,2]) * Z / self.cam_K[0,0]
                    Y = (cy_img - self.cam_K[1,2]) * Z / self.cam_K[1,1]
                    tvec = np.array([[X],[Y],[Z]], dtype=np.float32)
                    used_pnp = False
                else:
                    tvec = None

            # Fallback if PnP failed
            if tvec is None:
                fallback = self.fallback_dist(cx, cy, h, w, real_w, real_h)
                tvec = fallback.reshape(3,1)
                used_pnp = False

            # Draw bounding box & label
            box_color = self.class_colors.get(cls_name, (0,255,0))
            cv2.rectangle(annotated, (x1,y1), (x2,y2), box_color, 2)
            label_text = f"{cls_name} {conf:.2f}"
            (tw, th), baseline = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(annotated, (x1, y1 - th - baseline - 4), (x1 + tw + 4, y1), box_color, cv2.FILLED)
            cv2.putText(annotated, label_text, (x1+2, y1 - baseline - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)

            # Convert tvec (cm) to meters and from OpenCV camera coords to ROS optical coords
            Xc, Yc, Zc = (tvec.flatten() / 100.0)  # in meters
            # OpenCV camera frame: X right, Y down, Z forward
            # ROS camera optical: X forward, Y left, Z up
            x_ros = Zc
            y_ros = -Xc
            z_ros = -Yc

            # Annotate pose text
            text_col = (255,0,0) if used_pnp else (0,255,0)
            cv2.putText(annotated, f"X={x_ros:.2f}m", (x1, y1 - th - baseline - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_col, 2)
            cv2.putText(annotated, f"Y={y_ros:.2f}m", (x1, y1 - th - baseline - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_col, 2)
            cv2.putText(annotated, f"Z={z_ros:.2f}m", (x1, y1 - th - baseline - 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_col, 2)

            # Build Pose in camera frame (ROS optical)
            pose = Pose()
            pose.position.x = x_ros
            pose.position.y = y_ros
            pose.position.z = z_ros
            # Orientation: optional, but EKF likely uses only position
            if rvec is not None:
                # Convert rotation matrix from OpenCV coords to ROS optical frame
                R_cv, _ = cv2.Rodrigues(rvec)
                # Note: depending on your camera TF, you may need to permute axes here.
                # For simplicity, we skip complex orientation transforms and set identity:
                M = np.eye(4)
                M[:3,:3] = R_cv
                q = tfm.quaternion_from_matrix(M)
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]
            else:
                pose.orientation.w = 1.0

            rospy.logdebug(f"[YOLO] Detected in camera optical coords: X={x_ros:.2f}, Y={y_ros:.2f}, Z={z_ros:.2f}")

            # Transform pose to chassis::link
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header  # frame_id is camera frame
            pose_stamped.pose = pose
            try:
                if not self.tf_buffer.can_transform("chassis::link",
                                                    pose_stamped.header.frame_id,
                                                    rospy.Time(0),
                                                    rospy.Duration(0.2)):
                    rospy.logwarn("[YOLO] Cannot transform from %s to chassis::link yet." 
                                  % pose_stamped.header.frame_id)
                    continue
                tf_out = self.tf_buffer.lookup_transform("chassis::link",
                                                         pose_stamped.header.frame_id,
                                                         rospy.Time(0),
                                                         rospy.Duration(0.2))
                pose_chassis = tf2_geometry_msgs.do_transform_pose(pose_stamped, tf_out)
                # Append to PoseArray and labels_list
                poses.poses.append(pose_chassis.pose)
                labels_list.append(cls_name.split('_')[0])
                rospy.logdebug(f"[YOLO] Pose in chassis frame: x={pose_chassis.pose.position.x:.2f}, "
                               f"y={pose_chassis.pose.position.y:.2f}, z={pose_chassis.pose.position.z:.2f}")
            except Exception as e:
                rospy.logwarn(f"[YOLO] TF transform to chassis::link failed: {e}")
                continue

            # Publish TF for this sign in camera frame only once per timestamp
            tf_msg = TransformStamped()
            tf_msg.header = msg.header
            tf_msg.child_frame_id = f"sign_{i}"
            tf_msg.transform.translation.x = x_ros
            tf_msg.transform.translation.y = y_ros
            tf_msg.transform.translation.z = z_ros
            tf_msg.transform.rotation = pose.orientation
            last_stamp = self.last_tf_stamps.get(tf_msg.child_frame_id)
            if last_stamp != tf_msg.header.stamp:
                self.last_tf_stamps[tf_msg.child_frame_id] = tf_msg.header.stamp
                try:
                    self.tf_br.sendTransform(tf_msg)
                except Exception as e:
                    rospy.logwarn(f"[YOLO] sendTransform failed: {e}")

        # Publish annotated image
        self.last_annotated = annotated
        out_img = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        out_img.header = msg.header
        self.pub_img.publish(out_img)

        # Publish PoseArray & labels in lockstep
        poses.header.frame_id = "chassis::link"
        self.pub_poses.publish(poses)
        labels_msg = SignLabelArray()
        labels_msg.labels = labels_list
        self.pub_labels.publish(labels_msg)

    def spin(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.last_annotated is not None:
                cv2.imshow('Detections', self.last_annotated)
                cv2.waitKey(1)
            rate.sleep()

if __name__ == '__main__':
    node = YoloSignPoseNode()
    node.spin()
