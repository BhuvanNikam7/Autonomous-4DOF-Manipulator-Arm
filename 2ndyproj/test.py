import cv2 as cv
from cv2 import aruco
import numpy as np
import time

# Load calibration data
def calculate_average(readings):
    avg_x = np.mean([reading[0] for reading in readings])
    avg_y = np.mean([reading[1] for reading in readings])
    avg_z = np.mean([reading[2] for reading in readings])
    return avg_x, avg_y, avg_z

def capture_marker_data():
    def estimate_pose_single_marker(corners, marker_size):
        marker_points = np.array(
            [
                [-marker_size / 2, marker_size / 2, 0],
                [marker_size / 2, marker_size / 2, 0],
                [marker_size / 2, -marker_size / 2, 0],
                [-marker_size / 2, -marker_size / 2, 0],
            ],
            dtype=np.float32,
        )
        rvecs = []
        tvecs = []
        for c in corners:
            _, rvec, tvec = cv.solvePnP(
                marker_points, c, cam_mat, dist_coef, False, cv.SOLVEPNP_IPPE_SQUARE
            )
            rvecs.append(rvec)
            tvecs.append(tvec)
        return rvecs, tvecs

    calib_data_path = "../calib_data/MultiMatrix.npz"
    calib_data = np.load(calib_data_path)
    cam_mat = calib_data["camMatrix"]
    dist_coef = calib_data["distCoef"]

    # Marker settings
    MARKER_SIZE = 8  # centimeters
    marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    param_markers = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(marker_dict, param_markers)

    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return []

    # Buffer to store readings
    readings_buffer = []
    start_time = time.time()
    duration = 10  # Duration in seconds

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break

        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, _ = detector.detectMarkers(gray_frame)

        if marker_corners:
            rVec, tVec = estimate_pose_single_marker(marker_corners, MARKER_SIZE)
            for i, (ids, corners) in enumerate(zip(marker_IDs, marker_corners)):
                cv.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)
                corners = corners.reshape(4, 2)

                # Draw the pose of the marker
                cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
                
                # Get coordinates
                x, y, z = tVec[i][0][0], tVec[i][1][0], tVec[i][2][0]
                readings_buffer.append((x, y, z))

                # Ensure we only keep the last 20 readings
                if len(readings_buffer) > 20:
                    readings_buffer.pop(0)

        cv.imshow("frame", frame)
        
        if time.time() - start_time > duration:
            break
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv.destroyAllWindows()
    
    if len(readings_buffer) >= 20:
        average_coords = calculate_average(readings_buffer)
        print(f"Average coordinates - x: {average_coords[0]:.2f}, y: {average_coords[1]:.2f}, z: {average_coords[2]:.2f}")
    else:
        print("Not enough readings to calculate an average.")
    
    return average_coords  # Return the captured average coordinates