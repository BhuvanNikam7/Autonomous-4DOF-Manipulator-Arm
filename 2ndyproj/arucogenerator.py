import cv2
import numpy as np

# Ensure the aruco module is present in cv2
if not hasattr(cv2, 'aruco'):
    raise ImportError("OpenCV was installed without aruco module. Install opencv-contrib-python to use aruco module.")

# Define parameters
marker_id = 42  # ID of the marker
marker_size = 200  # Size of the marker in pixels
dictionary_id = cv2.aruco.DICT_5X5_250  # Dictionary ID

# Get the ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)

# Generate the marker image
marker_image = np.zeros((marker_size, marker_size), dtype=np.uint8)
marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

# Save the marker image
cv2.imwrite('aruco_marker.png', marker_image)

# Display the marker
cv2.imshow('ArUco Marker', marker_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
