import cv2
import numpy as np

# Ensure the aruco module is present in cv2
if not hasattr(cv2, 'aruco'):
    raise ImportError("OpenCV was installed without aruco module. Install opencv-contrib-python to use aruco module.")

# Load the camera matrix and distortion coefficients from a previous calibration
# Replace these with your actual calibration data
camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((5, 1))  # Assuming no lens distortion

# Initialize the dictionary and detector parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
detector_params = cv2.aruco.DetectorParameters()

# Create an ArUco detector
detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

# Capture video from the default camera
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect markers in the image
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)

    if ids is not None:
        # Draw detected markers
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Estimate pose of each marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)

        for rvec, tvec in zip(rvecs, tvecs):
            # Draw axis for each marker
            cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)
            print(f"Translation vector: {tvec}, Rotation vector: {rvec}")

    # Display the frame
    cv2.imshow('ArUco Marker Detection', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
