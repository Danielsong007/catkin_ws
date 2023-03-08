import sys
import cv2 

# set video id
if len(sys.argv) > 1:
    print('video', sys.argv[1])
    cap = cv2.VideoCapture(int(sys.argv[1]))
else:
    cap = cv2.VideoCapture(1)

while(1): 
    # get a image 
    ret, image = cap.read()
    if image is not None:
        cv2.imshow('Img', image)
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break
cap.release() 
cv2.destroyAllWindows()