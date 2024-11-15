## This is the main file to run the Transformer for testing
import cv2 as cv
from PIL import Image
from Transformation import get_limits


cap = cv.VideoCapture(0)
yellow = [0, 255, 255]


while True:
    ret, frame = cap.read()
    hsvImage = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
    lowerlimit, upperlimit = get_limits(color = color)
    mask = cv.inRange(hsvImage, lowerlimit, upperlimit)

    # Detect the object with the box
    mask_temp = Image.fromarray(mask)
    bbox = mask_temp.getbbox()
    if bbox is not None:
        x1, y1, x2, y2 = bbox
        frame = cv.rectangle(frame, (x1,y1), (x2,y2), (0, 255, 0), 5)
    print(bbox)
    
    cv.imshow('frame', frame)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()