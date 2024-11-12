import numpy as np
import cv2 as cv

img = cv.imread('/home/pi/hello-git/log-git/week10/관련 images_total/messi.jpg')
px = img[100, 100]
print(px)

blue = img[100, 100, 0]
print(blue)

img[100, 100] = [255, 255, 255]
print(img[100, 100])
print(img.shape)
print(img.size)
print(img.dtype)

img = cv.resize(img, (0, 0), fx=0.5, fy=0.5)
face = img[200:450, 600:900]
img[0:250, 0:300] = face
cv.imshow('face', face)
cv.waitKey(0)
cv.imshow('img+face', img)
cv.waitKey(0)

