import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while 1:
    ret ,frame =cap.read()
    # ret will return a true value if the frame exists otherwise False
    into_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # changing the color format from BGr to HSV
    # This will be used to create the mask

    #Blue
    Blue_L_limit = np.array([98, 50, 50])  # setting the blue lower limit
    Blue_U_limit = np.array([139, 255, 255])  # setting the blue upper limit

    #Red
    Red_L_limit = np.array([145,215,120])
    Red_U_limit = np.array([200,240,175])

    b_mask = cv2.inRange(into_hsv, Blue_L_limit, Blue_U_limit)
    r_mask = cv2.inRange(into_hsv, Red_L_limit,Red_U_limit)
    # creating the mask using inRange() function
    # this will produce an image where the color of the objects
    # falling in the range will turn white and rest will be black
    blue = cv2.bitwise_and(frame, frame, mask=b_mask)
    red = cv2.bitwise_and(frame, frame, mask=r_mask)
    # this will give the color to mask.
    cv2.imshow('Original', frame)  # to display the original frame
    cv2.imshow('Blue Detector', blue)  # to display the blue object output
    cv2.imshow("Red Detector", red) # To display the red object output

    if cv2.waitKey(1) == 27:
        break
    # this function will be triggered when the ESC key is pressed
    # and the while loop will terminate and so will the program
cap.release()

cv2.destroyAllWindows()
