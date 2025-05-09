import cv2
import time
import argparse
from ultralytics import YOLO
import numpy as np

class SignDetector:
    def __init__(self, model_path="/home/bb/Brain/src/hardware/camera/utils/best.pt", conf=0.5, device="cpu"):
        # Real-world sign dimensions (in cm)
        self.DEFAULT_SIGN_WIDTH = 6
        self.DEFAULT_SIGN_HEIGHT = 6
        self.HIGHWAY_SIGN_WIDTH = 4.2 

        # Camera calibration parameters for a Logitech webcam
        self.camera_matrix = np.array([[520.01502575, 0., 313.49750048],
                                       [0., 519.84156135, 246.65765466],
                                       [0., 0., 1.]])
        self.dist_coeffs = np.array([-0.1132961,   0.13205656,  0.00449707, -0.00077401, -0.1324293])
        
        # Load the YOLO model and set the device
        self.model = YOLO(model_path)
        # self.model.to(device)
        self.conf = conf

    @staticmethod
    def order_points(pts):
        """
        Order points in the following order:
        top-left, top-right, bottom-right, bottom-left.
        """
        # sort by x coordinate
        xSorted = pts[np.argsort(pts[:, 0]), :]
        leftMost = xSorted[:2, :]
        rightMost = xSorted[2:, :]
        # sort leftMost according to y coordinate to get top-left and bottom-left
        leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
        tl, bl = leftMost[0], leftMost[1]
        # sort rightMost according to y coordinate to get top-right and bottom-right
        rightMost = rightMost[np.argsort(rightMost[:, 1]), :]
        tr, br = rightMost[0], rightMost[1]
        return np.array([tl, tr, br, bl], dtype="float32")

    def get_sign_pose(self, roi, x_offset, y_offset, SIGN_WIDTH, SIGN_HEIGHT):
        """
        Process the ROI (region of interest) corresponding to the detected sign,
        extract a quadrilateral contour, and use solvePnP to estimate the pose.
        """
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        ret, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            return None, None, None

        # Select the largest contour assuming it is the sign.
        c = max(contours, key=cv2.contourArea)
        epsilon = 0.02 * cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, epsilon, True)
        
        # Four corners for pose estimation.
        if len(approx) != 4:
            return None, None, None

        pts = approx.reshape(4, 2).astype("float32")
        ordered_pts = SignDetector.order_points(pts)
        image_points = ordered_pts + np.array([x_offset, y_offset], dtype="float32")

        # Corresponding 3D object points (assume the sign lies on the plane z=0)
        object_points = np.array([
            [0, 0, 0],
            [SIGN_WIDTH, 0, 0],
            [SIGN_WIDTH, SIGN_HEIGHT, 0],
            [0, SIGN_HEIGHT, 0]
        ], dtype=np.float32)

        success, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coeffs)
        if not success:
            return None, None, None
        return rvec, tvec, image_points

    def xyz_of_sign(self, x_min, y_min, pixel_width, pixel_height, label): 
        """
        Compute the 3D coordinates (x, y, z) of the sign using the pinhole model.
        """
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        # Center of the bounding box
        u = x_min + (pixel_width / 2.0)
        v = y_min + (pixel_height / 2.0)

        SIGN_WIDTH = self.DEFAULT_SIGN_WIDTH 
        if "Highway" in label:
            SIGN_WIDTH = self.HIGHWAY_SIGN_WIDTH 

        # Depth (z) using both width and height estimates and averaging them
        z_width = (fx * SIGN_WIDTH) / pixel_width
        z_height = (fy * self.DEFAULT_SIGN_HEIGHT) / pixel_height
        z = (z_width + z_height) / 2.0

        # Convert the center pixel (u, v) into 3D space using the pinhole model
        x = ((u - cx) / fx) * z
        y = ((v - cy) / fy) * z

        return x, y, z

    def detect_sign(self, frame):
        """
        Given an input frame, this function undistorts and resizes the image,
        runs the YOLO model, and then computes the sign's pose. It returns a
        tuple containing a list of detected sign information (each with label, x, y, z)
        and the annotated frame.
        """
        # Undistort the frame using the camera matrix and distortion coefficients.
        frame_undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, None, self.camera_matrix)
        resized_img = cv2.resize(frame_undistorted, (640, 480))

        # YOLO detection on the undistorted & resized frame
        result = self.model(resized_img, conf=self.conf, verbose=False)[0]  
        detections = []

 

        if result.boxes is not None and len(result.boxes.xyxy) > 0:
            for i in range(len(result.boxes.xyxy)):
                # Extract bounding box coordinates in the 640x480 image space
                x1_res, y1_res, x2_res, y2_res = result.boxes.xyxy[i]
                conf = float(result.boxes.conf[i])
                cls_id = int(result.boxes.cls[i])
                label = self.model.names[cls_id]


                x1 = int(x1_res)
                y1 = int(y1_res)
                x2 = int(x2_res)
                y2 = int(y2_res)

                # Calculate ROI and bounding box dimensions
                roi = resized_img[y1:y2, x1:x2]
                pixel_width = x2 - x1
                pixel_height = y2 - y1


                if "Highway" in label:
                    sign_width = self.HIGHWAY_SIGN_WIDTH
                else:
                    sign_width = self.DEFAULT_SIGN_WIDTH
                sign_height = self.DEFAULT_SIGN_HEIGHT

                rvec, tvec, image_points = self.get_sign_pose(roi, x1, y1, sign_width, sign_height)
                if rvec is not None and tvec is not None:
                    # Use solvePnP output (blue text and circles)
                    x_pose, y_pose, z_pose = tvec.flatten()
                    cv2.putText(resized_img, f"X: {x_pose:.2f}cm", (x1, y1 - 35),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    cv2.putText(resized_img, f"Y: {y_pose:.2f}cm", (x1, y1 - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    cv2.putText(resized_img, f"Z: {z_pose:.2f}cm", (x1, y1 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    #print(f"Sign {i+1}: {label} at X: {x_pose:.2f}cm, Y: {y_pose:.2f}cm, Z: {z_pose:.2f}cm")
                    # Draw the detected corners.
                    for pt in image_points:
                        pt = tuple(pt.astype(int))
                        cv2.circle(resized_img, pt, 5, (255, 0, 0), -1)
                    detections.append({"label": label, "x": x_pose, "y": y_pose, "z": z_pose})
                else:
                    # Fallback 3D coordinate estimation (green text)
                    x, y, z = self.xyz_of_sign(float(x1), float(y1), float(pixel_width), float(pixel_height), label)
                    cv2.putText(resized_img, f"X: {x:.2f}cm", (x1, y1 - 35),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(resized_img, f"Y: {y:.2f}cm", (x1, y1 - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(resized_img, f"Z: {z:.2f}cm", (x1, y1 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    #print(f"Sign {i+1}: {label} at X: {x:.2f}cm, Y: {y:.2f}cm, Z: {z:.2f}cm")
                    detections.append({"label": label, "x": x, "y": y, "z": z})
                cv2.rectangle(resized_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(resized_img, f"{label} {conf:.2f}", (x1, y1 - 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        return detections, resized_img


def main():
    parser = argparse.ArgumentParser(
        description="YOLO detection using a USB camera with option to display live or save output."
    )
    parser.add_argument(
        "--source",
        type=str,
        default="0",
        help="Camera source. Use an integer index (e.g., 0) for a USB camera or a video file path."
    )
    parser.add_argument(
        "--output",
        type=str,
        default="",
        help="If provided, the output video (with detections) will be saved to this file instead of being displayed live."
    )
    parser.add_argument(
        "--model",
        type=str,
        default="best.pt",
        help="Path to the YOLO model weights (default: best.pt)."
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.5,
        help="Confidence threshold for detections (default: 0.5)."
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda",
        help="Decide which device the model is run on, (default: 'cuda') or 'cpu'"
    )

    args = parser.parse_args()

    # Determine video source (USB camera index or file path)
    try:
        source = int(args.source)
    except ValueError:
        source = args.source

    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print(f"Error: Could not open video source: {source}")
        return

    # Set up VideoWriter if output is specified
    writer = None
    if args.output:
        width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps    = cap.get(cv2.CAP_PROP_FPS)
        if fps <= 0:
            fps = 20.0
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(args.output, fourcc, fps, (width, height))
        print(f"Output will be saved to {args.output}")

    times = []
    frame_count = 0

    # Create an instance of SignDetector with local parameters
    detector = SignDetector(model_path=args.model, conf=args.conf, device=args.device)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame from source.")
            break

        start = time.time()
        detections, processed_frame = detector.detect_sign(frame)
        end = time.time()
        times.append(end - start)
        frame_count += 1

        if writer:
            writer.write(processed_frame)
        else:
            # cv2.imshow("USB Camera YOLO Detection", processed_frame)
            # Press 'q' to exit live stream mode
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print(f"Quitting live display. Processed {frame_count} frames.")
                break

    cap.release()
    if writer:
        writer.release()
        print(f"Saved processed video to {args.output}")
    cv2.destroyAllWindows()

    total_time = sum(times)
    if total_time > 0:
        print(f"Average FPS: {frame_count / total_time:.2f}")


if __name__ == "__main__":
    main()
