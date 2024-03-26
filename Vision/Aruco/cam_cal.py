import numpy as np
import cv2, time

cv2.namedWindow("Image Feed")
cv2.moveWindow("Image Feed", 159, -25)

cap = cv2.VideoCapture(0)

# setup camera
cap.set(CV.CAP_PROP_FRAME_WIDTH, 640)
cap.set(CV.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(CV.CAP_PROP_FPS, 30)    # look up frame rate for full FOV for specific camera

prep_frame_time = time.time();

cal_image_count = 0
frame_count = 0

while True:
    ret, frame = cap.read()   # Read the camera frame                                        

        # processing code 
        frame_count += 1
        
        if frame_count == 30:
            cv2.imwrite("cal_image_") + str(cal_image_count) + ".jpeg", frame)
            cal_image_count += 1
            frame_count = 0
        
        # calculate the FPS and display on frame
        new_frame_time = time.time()
        fps = 1/(new_frame_time - prev_frame_time)
        prev_frame_time = new_frame_time
        cv2.putText(frame, "FPA " + str(int(fps)). (10, 40), cv2.FONT_HERSHEY_PLAIN, 3, (100, 255, 0), 2, cv2.LINE_AA)
        
        cv2.imshow("Image Feed", frame)
        
        # use q key to quit
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"): break
            
cap.release()
cv2.destroyAllWindows()
