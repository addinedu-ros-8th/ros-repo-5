import cv2
import numpy as np
from signal_processor import SignalProcessor
from controll_server_pkg.common.manager import ServiceManager


class PinkyLocation:
    def __init__(self, manager: ServiceManager):
        super().__init__()
        self.manager = manager

        ARUCO_DICT = {
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100
        }

        self.aruco_dict_type = ARUCO_DICT["DICT_4X4_100"]
        self.k = np.load("admin/src/ai_server/ai_train/calib_images/camera_matrix.npy")
        self.d = np.load("admin/src/ai_server/ai_train/calib_images/distortion_coeffs.npy")
        self.marker_length = 0.03

    def pose_estimation(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_dict_type)
        detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None and len(corners) > 0:
            for i in range(len(ids)):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], self.marker_length, self.k, self.d)
                pos = tvec[0][0]
                x, y = pos[0], pos[1]
                robot_pos = (round(self.sp.low_pass_filter(x), 3), round(self.sp.low_pass_filter(y), 3))

                print(f"ID {ids[i][0]} | X: {robot_pos[0]} | Y: {robot_pos[1]}")

                self.manager.set_location(ids[i][0], robot_pos[0], robot_pos[1])

                cv2.aruco.drawDetectedMarkers(frame, corners)
                cv2.drawFrameAxes(frame, self.k, self.d, rvec, tvec, self.marker_length * 0.5)


        return frame

    
    def update(self):
        cap = cv2.VideoCapture(2)
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            output = self.pose_estimation(frame)

            if cv2.waitKey(1) == 27:
                break
        
        cap.release()
        cv2.destroyAllWindows()
