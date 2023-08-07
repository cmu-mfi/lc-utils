import cv2
import numpy as np

img1 = cv2.imread('image1.png', cv2.IMREAD_GRAYSCALE)
img2 = cv2.imread('image2.png', cv2.IMREAD_GRAYSCALE)

print('Red max: {}'.format(np.max(img1)))
print('Red min: {}'.format(np.min(img1)))
print('Blue max: {}'.format(np.max(img2)))
print('Blue min: {}'.format(np.min(img2)))

cv2.imshow('Img1', img1)
cv2.waitKey(0)
cv2.imshow('Img2', img2)
cv2.waitKey(0)
