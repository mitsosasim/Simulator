#!/usr/bin/env python3
"""
YoloSignPoseNode

A ROS node that uses YOLO (via torch.hub) to detect traffic signs in a camera image,
estimates their 3D pose in the camera frame, converts to ROS camera-optical coords,
transforms into chassis::link, and publishes both a PoseArray and sign labels.
Also publishes annotated image and TF frames for each sign (once per timestamp).

Added: basic outlier filtering so that a detection is only published if its estimated
pose is close (within a threshold) to the previous frame's detection(s) of the same class,
to suppress isolated far-away outliers.
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

        # Outlier filter threshold (meters)
        # Only publish detections whose pose is within this distance of any previous detection of same class.
        self.pose_filter_threshold = rospy.get_param('~pose_filter_threshold', 0.08)

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

        # Store previous accepted detections: list of dict {'class': cls_name, 'pos': np.array([x,y,z])}
        self.prev_detections = []

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
        # Order 4 points: top-left, top-right, bottom-right, bottom-left
        xSorted = pts[np.argsort(pts[:,0]),:]
        left, right = xSorted[:2], xSorted[2:]
        tl, bl = left[np.argsort(left[:,1])]
        tr, br = right[np.argsort(right[:,1])]
        return np.array([tl, tr, br, bl], dtype=np.float32)

    def fallback_dist(self, cx, cy, pix_h, pix_w, real_w, real_h):
        # Simple pinhole fallback: returns (X, Y, Z) in camera coords (cm)
        Z_h = (self.cam_K[1,1] * real_h) / pix_h if pix_h>0 else None
        Z_w = (self.cam_K[0,0] * real_w) / pix_w if pix_w>0 else None
        if Z_h is None or Z_w is None:
            return np.array([0,0,0], dtype=np.float32)
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

        # Will accumulate accepted detections of this frame for next-frame filtering:
        curr_detections = []

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
            roi_bgr = frame_u[y1:y1+h, x1:x1+w]
            if roi_bgr.size == 0:
                continue
            roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)

            # Color masks for each sign type
            mask = None
            try:
                if shape == 'rectangle' and cls_name in ['oneway', 'crosswalk', 'parking']:
                    # Blue mask (approx)
                    lower = np.array([100, 80, 40])
                    upper = np.array([130, 255, 255])
                    mask = cv2.inRange(roi_hsv, lower, upper)
                elif shape == 'rectangle' and cls_name in ['enterhighway', 'exithighway']:
                    # Green mask
                    lower = np.array([40, 60, 40])
                    upper = np.array([90, 255, 255])
                    mask = cv2.inRange(roi_hsv, lower, upper)
                elif shape == 'circle' and cls_name == 'noentry':
                    # Red mask
                    lower1 = np.array([0, 70, 50])
                    upper1 = np.array([10, 255, 255])
                    lower2 = np.array([170, 70, 50])
                    upper2 = np.array([180, 255, 255])
                    mask1 = cv2.inRange(roi_hsv, lower1, upper1)
                    mask2 = cv2.inRange(roi_hsv, lower2, upper2)
                    mask = cv2.bitwise_or(mask1, mask2)
                else:
                    # Fallback: use grayscale threshold
                    gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
                    _, mask = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY)
            except Exception as e:
                rospy.logwarn(f"[YOLO] Color mask failed for {cls_name}: {e}")
                gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
                _, mask = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY)

            # Morphological closing to fill gaps in the border
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Edge detection on mask
            edges = cv2.Canny(mask, 50, 150)
            # Optional dilate to strengthen thin edges
            edges = cv2.dilate(edges, np.ones((3,3), np.uint8), iterations=1)
            cnts_info = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(cnts_info) == 3:
                _, cnts, hierarchy = cnts_info
            else:
                cnts, hierarchy = cnts_info
            if not cnts or hierarchy is None:
                rospy.logdebug(f"[YOLO] No contours for {cls_name}")
                selected_contour = None
            else:
                # Shape-based contour selection
                selected_contour = None
                best_score = None
                roi_area = float(w * h)
                # Also track largest contour for fallback
                largest_contour = None
                largest_area = 0.0
                for idx_c, c in enumerate(cnts):
                    area = cv2.contourArea(c)
                    if area > largest_area:
                        largest_area = area
                        largest_contour = c.copy()
                    # Skip tiny
                    if area < 0.05 * roi_area:
                        continue
                    peri = cv2.arcLength(c, True)
                    if peri <= 0:
                        continue
                    approx = cv2.approxPolyDP(c, 0.02 * peri, True)
                    num_vertices = len(approx)
                    # Evaluate based on shape
                    # Helper for angle cosine
                    def angle_cos(u, v):
                        dot = u.dot(v); nu = np.linalg.norm(u); nv = np.linalg.norm(v)
                        if nu*nv == 0: return 1.0
                        return dot/(nu*nv)
                    if shape == 'rectangle':
                        # Prefer approx with 4 vertices
                        if num_vertices == 4:
                            pts = approx.reshape(-1,2).astype(np.float32) + np.array([x1, y1], dtype=np.float32)
                            ordered = self.order_pts(pts)
                            # Edge vectors
                            v0 = ordered[1] - ordered[0]
                            v1 = ordered[2] - ordered[1]
                            v2 = ordered[3] - ordered[2]
                            v3 = ordered[0] - ordered[3]
                            c0 = abs(angle_cos(v0, v1))
                            c1 = abs(angle_cos(v1, v2))
                            c2 = abs(angle_cos(v2, v3))
                            c3 = abs(angle_cos(v3, v0))
                            score = c0 + c1 + c2 + c3
                            # Check bounding-box proximity to ROI edges
                            xs = pts[:,0] - x1; ys = pts[:,1] - y1
                            x_min, x_max = xs.min(), xs.max()
                            y_min, y_max = ys.min(), ys.max()
                            # require contour near ROI border (<=10% margin)
                            border_ok = (x_min < 0.1*w and x_max > 0.9*w) or (y_min < 0.1*h and y_max > 0.9*h)
                            if score < 0.5 and border_ok:
                                # choose minimal score, then larger area
                                key = (score, -area)
                                if selected_contour is None or key < best_score:
                                    selected_contour = c.copy()
                                    best_score = key
                    elif shape == 'circle':
                        # circularity metric
                        circ = 4*np.pi*area/(peri*peri)
                        if circ > 0.6:
                            # score by closeness to 1, and area threshold
                            score = abs(1.0 - circ)
                            # require area sufficiently large
                            if area > 0.1*roi_area:
                                key = (score, abs(area - 0.5*roi_area))
                                if selected_contour is None or key < best_score:
                                    selected_contour = c.copy()
                                    best_score = key
                    elif shape == 'triangle':
                        if num_vertices == 3:
                            # prefer larger area
                            key = (-area,)
                            if selected_contour is None or key < best_score:
                                selected_contour = c.copy()
                                best_score = key
                    elif shape == 'diamond':
                        if num_vertices == 4:
                            pts = approx.reshape(-1,2).astype(np.float32) + np.array([x1, y1], dtype=np.float32)
                            ordered = self.order_pts(pts)
                            v0 = ordered[1] - ordered[0]
                            v1 = ordered[2] - ordered[1]
                            v2 = ordered[3] - ordered[2]
                            v3 = ordered[0] - ordered[3]
                            c0 = abs(angle_cos(v0, v1))
                            c1 = abs(angle_cos(v1, v2))
                            c2 = abs(angle_cos(v2, v3))
                            c3 = abs(angle_cos(v3, v0))
                            score = c0 + c1 + c2 + c3
                            # allow more slack
                            if score < 0.7:
                                key = (score, -area)
                                if selected_contour is None or key < best_score:
                                    selected_contour = c.copy()
                                    best_score = key
                    # else: skip shape-based for unknown shapes
                # If none selected by shape, fallback to largest contour
                if selected_contour is None and largest_contour is not None:
                    selected_contour = largest_contour.copy()
                    rospy.logdebug(f"[YOLO] No shape-matching contour for {cls_name}, using largest contour area={largest_area:.1f}")
                elif selected_contour is not None:
                    rospy.logdebug(f"[YOLO] Selected contour for {cls_name} with score={best_score}")

            # Now perform PnP or fallback using selected_contour
            rvec = None; tvec = None; used_pnp = False
            if 'selected_contour' in locals() and selected_contour is not None:
                c = selected_contour
                # For rectangle: use minAreaRect on contour
                if shape == 'rectangle':
                    try:
                        rect = cv2.minAreaRect(c)
                        box = cv2.boxPoints(rect).astype(np.float32)
                        # Transform box points to image coords
                        img_pts = self.order_pts(box + np.array([x1, y1], dtype=np.float32))
                        obj_pts = np.array([[0,0,0], [real_w,0,0], [real_w,real_h,0], [0,real_h,0]], dtype=np.float32)
                        ok, rvec_temp, tvec_temp, _ = cv2.solvePnPRansac(
                            obj_pts, img_pts, self.cam_K, self.dist,
                            reprojectionError=8.0, iterationsCount=100,
                            confidence=0.99, flags=cv2.SOLVEPNP_ITERATIVE
                        )
                        if ok:
                            rvec, tvec = rvec_temp, tvec_temp
                            used_pnp = True
                            rospy.logdebug(f"[YOLO] PnP rectangle succeeded for {cls_name}")
                        else:
                            rospy.logdebug(f"[YOLO] PnP rectangle failed (ok=False) for {cls_name}")
                    except cv2.error as e:
                        rospy.logwarn(f"[YOLO] PnP rectangle exception: {e}")
                elif shape == 'triangle':
                    if selected_contour is not None:
                        # get approx pts for triangle
                        peri = cv2.arcLength(c, True)
                        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
                        if len(approx) == 3:
                            pts = approx.reshape(-1,2).astype(np.float32) + np.array([x1, y1], dtype=np.float32)
                            obj_pts = np.array([[0,0,0], [real_w,0,0], [real_w/2, real_h, 0]], dtype=np.float32)
                            try:
                                ok, rvec_temp, tvec_temp = cv2.solvePnP(
                                    obj_pts, pts, self.cam_K, self.dist,
                                    flags=cv2.SOLVEPNP_ITERATIVE
                                )
                                if ok:
                                    rvec, tvec = rvec_temp, tvec_temp
                                    used_pnp = True
                                    rospy.logdebug(f"[YOLO] PnP triangle succeeded for {cls_name}")
                            except cv2.error as e:
                                rospy.logwarn(f"[YOLO] PnP triangle exception: {e}")
                elif shape == 'diamond':
                    try:
                        peri = cv2.arcLength(c, True)
                        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
                        if len(approx) == 4:
                            pts = approx.reshape(-1,2).astype(np.float32) + np.array([x1, y1], dtype=np.float32)
                            img_pts = self.order_pts(pts)
                            w2, h2 = real_w/2.0, real_h/2.0
                            obj_pts = np.array([[0,-h2,0], [w2,0,0], [0,h2,0], [-w2,0,0]], dtype=np.float32)
                            ok, rvec_temp, tvec_temp = cv2.solvePnP(obj_pts, img_pts, self.cam_K, self.dist)
                            if ok:
                                rvec, tvec = rvec_temp, tvec_temp
                                used_pnp = True
                                rospy.logdebug(f"[YOLO] PnP diamond succeeded for {cls_name}")
                    except cv2.error as e:
                        rospy.logwarn(f"[YOLO] PnP diamond exception: {e}")
                elif shape == 'circle':
                    # Fit circle
                    try:
                        (cx_img, cy_img), radius = cv2.minEnclosingCircle(c)
                        if radius > 1e-3:
                            # center coordinates need ROI offset
                            X = (cx_img + x1 - self.cam_K[0,2]) * ((self.cam_K[1,1] * real_h) / (2*radius)) / self.cam_K[0,0]
                            Y = (cy_img + y1 - self.cam_K[1,2]) * ((self.cam_K[1,1] * real_h) / (2*radius)) / self.cam_K[1,1]
                            Z = (self.cam_K[1,1] * real_h) / (2*radius)
                            tvec = np.array([[X],[Y],[Z]], dtype=np.float32)
                            used_pnp = False
                    except cv2.error as e:
                        rospy.logwarn(f"[YOLO] Circle fallback exception: {e}")
            # If no tvec from PnP or circle
            if tvec is None:
                # fallback pinhole
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
            # Orientation: identity or from rvec if needed
            if rvec is not None:
                try:
                    R_cv, _ = cv2.Rodrigues(rvec)
                    M = np.eye(4)
                    M[:3,:3] = R_cv
                    q = tfm.quaternion_from_matrix(M)
                    pose.orientation.x = q[0]
                    pose.orientation.y = q[1]
                    pose.orientation.z = q[2]
                    pose.orientation.w = q[3]
                except Exception:
                    pose.orientation.w = 1.0
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
                    rospy.logwarn(f"[YOLO] Cannot transform from {pose_stamped.header.frame_id} to chassis::link yet.")
                    continue
                tf_out = self.tf_buffer.lookup_transform("chassis::link",
                                                         pose_stamped.header.frame_id,
                                                         rospy.Time(0),
                                                         rospy.Duration(0.2))
                pose_chassis = tf2_geometry_msgs.do_transform_pose(pose_stamped, tf_out)
                # Before appending, apply outlier filter:
                # Check previous detections for this class:
                curr_pos = np.array([
                    pose_chassis.pose.position.x,
                    pose_chassis.pose.position.y,
                    pose_chassis.pose.position.z
                ], dtype=np.float32)
                accept = False
                # Find previous poses of same class
                prev_positions = [d['pos'] for d in self.prev_detections if d['class'] == cls_name]
                if not prev_positions:
                    # no prior: accept
                    accept = True
                else:
                    # compute distances to all prior of same class:
                    dists = [np.linalg.norm(curr_pos - p) for p in prev_positions]
                    min_dist = min(dists) if dists else float('inf')
                    if min_dist <= self.pose_filter_threshold:
                        accept = True
                    else:
                        rospy.logdebug(f"[YOLO] Filtering outlier for {cls_name}: dist {min_dist:.2f} > threshold {self.pose_filter_threshold:.2f}")
                if accept:
                    poses.poses.append(pose_chassis.pose)
                    labels_list.append(cls_name.split('_')[0])
                    # store for next frame
                    curr_detections.append({'class': cls_name, 'pos': curr_pos})
                    rospy.logdebug(f"[YOLO] Pose in chassis frame accepted: class={cls_name}, x={curr_pos[0]:.2f}, y={curr_pos[1]:.2f}, z={curr_pos[2]:.2f}")
                # else: skip publishing this detection
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

        # After processing all detections in this frame, update prev_detections:
        self.prev_detections = curr_detections

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
