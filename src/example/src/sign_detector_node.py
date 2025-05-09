#!/usr/bin/env python3
import rospy, cv2, numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from ultralytics import YOLO
import tf2_ros
import torch

class YoloSignPoseNode:
    def __init__(self):
        rospy.init_node('yolo_sign_pose_node')
        # params
        model_path = rospy.get_param('~model', 'best.pt')
        conf       = rospy.get_param('~conf',   0.5)
        device     = rospy.get_param('~device','cpu')
        self.sign_real_height = 6.0  # cm
        self.sign_real_width  = 6.0  # cm

        rospy.loginfo(f"[YOLO] CUDA available? {torch.cuda.is_available()}")

        # load YOLO
        self.model = YOLO(model_path)
        self.model.conf = conf
        if 'cuda' in device:
            self.model.to(device)
        params = list(self.model.model.parameters())
        rospy.loginfo(f"[YOLO] Model parameters on device: {params[0].device}")

        # cv_bridge + tf broadcaster
        self.bridge = CvBridge()
        self.tf_br  = tf2_ros.TransformBroadcaster()

        # camera intrinsics
        self.cam_K = None
        self.dist  = None
        rospy.Subscriber('/automobile/camera_info', CameraInfo,
                         self.info_cb, queue_size=1)

        # main subscription & pubs
        self.sub_img = rospy.Subscriber('/automobile/image_raw', Image,
                                        self.image_cb,
                                        queue_size=1,
                                        buff_size=4*1024*1024)
        self.pub_img   = rospy.Publisher('/automobile/image_annotated',
                                         Image, queue_size=1)
        self.pub_poses = rospy.Publisher('/sign_detector/sign_poses',
                                         PoseArray, queue_size=1)

        # OpenCV windows
        cv2.namedWindow('Detections', cv2.WINDOW_NORMAL)
        self.last_annotated = None

        # cleanup on shutdown
        rospy.on_shutdown(self.on_shutdown)

    def on_shutdown(self):
        cv2.destroyAllWindows()
        rospy.loginfo("[YOLO] Closed OpenCV windows")

    def info_cb(self, msg: CameraInfo):
        if self.cam_K is None:
            self.cam_K = np.array(msg.K).reshape(3,3)
            self.dist  = np.array(msg.D)
            rospy.loginfo(f"[PoseNode] Cam intrinsics fx={self.cam_K[0,0]:.1f}, "
                          f"fy={self.cam_K[1,1]:.1f}, cx={self.cam_K[0,2]:.1f}, "
                          f"cy={self.cam_K[1,2]:.1f}")

    @staticmethod
    def order_pts(pts):
        xSorted = pts[np.argsort(pts[:,0]),:]
        left, right = xSorted[:2], xSorted[2:]
        tl, bl = left[np.argsort(left[:,1])]
        tr, br = right[np.argsort(right[:,1])]
        return np.array([tl,tr,br,bl],dtype=np.float32)

    def solve_pnp(self, frame, x1,y1,w,h, real_w, real_h):
        roi = frame[y1:y1+h, x1:x1+w]
        gray = cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY)
        _,th = cv2.threshold(gray,100,255,cv2.THRESH_BINARY)
        cnts,_ = cv2.findContours(th,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        if not cnts: return None,None
        c = max(cnts, key=cv2.contourArea)
        eps = 0.02*cv2.arcLength(c,True)
        approx = cv2.approxPolyDP(c,eps,True)
        if len(approx)!=4: return None,None
        pts = approx.reshape(4,2).astype(np.float32) + np.array([x1,y1])
        img_pts = self.order_pts(pts)
        obj_pts = np.array([[0,0,0],
                            [real_w,0,0],
                            [real_w,real_h,0],
                            [0,real_h,0]],dtype=np.float32)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self.cam_K, self.dist)
        return (rvec,tvec) if ok else (None,None)

    def fallback_dist(self, cx,cy,pix_h):
        Z = (self.cam_K[1,1]*self.sign_real_height)/pix_h  # cm
        X = (cx - self.cam_K[0,2]) * Z / self.cam_K[0,0]
        Y = (cy - self.cam_K[1,2]) * Z / self.cam_K[1,1]
        return np.array([X,Y,Z],dtype=np.float32)

    def image_cb(self, msg: Image):
        if self.cam_K is None:
            return

        # decode & undistort
        frame       = self.bridge.imgmsg_to_cv2(msg,'bgr8')
        frame_u     = cv2.undistort(frame, self.cam_K, self.dist, None, self.cam_K)

        # YOLO inference
        res         = self.model(frame_u)[0]
        annotated   = res.plot()

        # poses container
        poses = PoseArray(header=msg.header)

        # annotate each detection
        for i,(box,conf,cls) in enumerate(zip(res.boxes.xyxy,
                                              res.boxes.conf,
                                              res.boxes.cls)):
            x1,y1,x2,y2 = map(int,box)
            w,h = x2-x1, y2-y1
            cx,cy = x1+w/2, y1+h/2

            # PnP
            real_w = real_h = self.sign_real_height
            rvec,tvec = self.solve_pnp(frame_u, x1,y1,w,h,real_w,real_h)
            if tvec is None:
                t = self.fallback_dist(cx,cy,h)
                rvec, tvec = None, t.reshape(3,1)

            # draw box
            cv2.rectangle(annotated,(x1,y1),(x2,y2),(0,255,0),2)

            # compute meters
            x_m = tvec[0,0]/100.0
            y_m = tvec[1,0]/100.0
            z_m = tvec[2,0]/100.0

            # annotate X,Y,Z 5px apart above box
            cv2.putText(annotated, f"X={x_m:.2f} m", (x1, y1-25),  cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
            cv2.putText(annotated, f"Y={y_m:.2f} m", (x1, y1-50),  cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
            cv2.putText(annotated, f"Z={z_m:.2f} m", (x1, y1-75),  cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)

            # build Pose
            p = Pose()
            p.position.x = x_m
            p.position.y = y_m
            p.position.z = z_m
            if rvec is not None:
                R,_ = cv2.Rodrigues(rvec)
                M    = np.eye(4); M[:3,:3]=R
                import tf.transformations as tfm
                q    = tfm.quaternion_from_matrix(M)
                p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w = q
            else:
                p.orientation.w = 1.0
            poses.poses.append(p)

            # broadcast TF
            tf = TransformStamped()
            tf.header         = msg.header
            tf.child_frame_id = f"sign_{i}"
            tf.transform.translation.x = x_m
            tf.transform.translation.y = y_m
            tf.transform.translation.z = z_m
            tf.transform.rotation       = p.orientation
            self.tf_br.sendTransform(tf)

        # show annotated image live
        self.last_annotated = annotated.copy()
        # cv2.imshow('Detections', annotated)
        # cv2.waitKey(1)

        # publish annotated image & poses
        out_img = self.bridge.cv2_to_imgmsg(annotated,'bgr8')
        out_img.header = msg.header
        self.pub_img.publish(out_img)
        self.pub_poses.publish(poses)

if __name__ == '__main__':
    node = YoloSignPoseNode()

    rate = rospy.Rate(30)  # match your camera frame rate
    while not rospy.is_shutdown():
        if node.last_annotated is not None:
            cv2.imshow('Detections', node.last_annotated)
            # must call waitKey from main thread to update the window
            cv2.waitKey(1)
        rate.sleep()