import cv2 as cv
from cv2 import aruco
import numpy as np
import time

# Load calibration data
calib_data_path = "../calib_data/MultiMatrix.npz"
calib_data = np.load(calib_data_path)
cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]

# Marker settings
MARKER_SIZE = 8  # centimeters
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
param_markers = aruco.DetectorParameters()
cap = cv.VideoCapture(0)

# Buffer to store readings
readings_buffer = []

def my_estimatePoseSingleMarkers(corners, marker_size):
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
    for i, c in enumerate(corners):
        nada, R, t = cv.solvePnP(
            marker_points, c, cam_mat, dist_coef, False, cv.SOLVEPNP_IPPE_SQUARE
        )
        rvecs.append(R)
        tvecs.append(t)
    return rvecs, tvecs


def calculate_average(readings):
    avg_x = np.mean([reading[0] for reading in readings])
    avg_y = np.mean([reading[1] for reading in readings])
    avg_z = np.mean([reading[2] for reading in readings])
    return (avg_x, avg_y, avg_z)

# Start the timer
start_time = time.time()
duration = 10  # Duration in seconds

while True:
    ret, frame = cap.read()
    if not ret:
        break
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    detector = cv.aruco.ArucoDetector(marker_dict, param_markers)
    marker_corners, marker_IDs, _ = detector.detectMarkers(gray_frame)
    if marker_corners:
        rVec, tVec = my_estimatePoseSingleMarkers(marker_corners, MARKER_SIZE)
        for i, (ids, corners) in enumerate(zip(marker_IDs, marker_corners)):
            cv.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)
            corners = corners.reshape(4, 2)
            top_right = tuple(corners[0].astype(int))

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

cap.release()
cv.destroyAllWindows()

# Calculate and print the average after the loop ends
if len(readings_buffer) == 20:
    average_coords = calculate_average(readings_buffer)
    #print(f"Average coordinates - x: {average_coords[0]:.2f}, y: {average_coords[1]:.2f}, z: {average_coords[2]:.2f}")
else:
    print("Not enough readings to calculate an average.")
