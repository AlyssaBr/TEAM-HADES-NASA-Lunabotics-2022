import numpy as np
import cv2
import cv2.aruco as aruco

marker_size = 100

with open('camera_cal.npy', 'rb') as f:
  camera_matrix = np.load(f)
  camera_distortion = np.load(f)
 
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4x4_250)

cap = cv2.VideoCapture(0)

camera_width = 640
camera_height = 480
camera_frame_rate = 40

ret, frame = cap.read()     # grab frame

gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

# Find aruco marker in the image
corners, ids, rejected = aruco.detectMarkers(gray_frame, aruco_dict, camera_matrix, camera_distortion)

if ids is not None:
  aruco.drawDetectedMarkers(frame, corners)
  
  # get pose of marker, rotation and translation vectors
  rvec, tvec, _objPoints = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
  
  # draw axis and write ids on markers
  for marker in range(len(ids)):
    aruco.drawAzis(frame, camera_matrix, camera_distortion, rvec[marker], tvec[marker], 100)
    cv2.putText(frame, str(ids[marker][0]), (int(corners[marker][0][0][0]) - 30, int(corners[marker][0][0][1])), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 0), 2, cv2.LINE_AA)

cv2.imshow('frame', frame)

cv2.waitKey(0)

cap.release()
cv2.destroyAllWindows()
