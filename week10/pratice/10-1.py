import cv2 as cv
import sys

img = cv.imread("/home/pi/hello-git/log-git/week10/관련 images_total/snight.png")

if img is None:
    sys.exit("Could not read the image")

cv.imshow("Display window", img)
k = cv.waitKey(0)

if k == ord("s"):
    cv.imwrite("snight.png", img)