import cv2
import numpy as np

# Load image, grayscale, Gaussian blur, threshold
image = cv2.imread('/home/gouda/segmentation/clutter_annotation_tool/seg_mask.png')
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
thresh = cv2.threshold(gray, 5, 255, cv2.THRESH_BINARY)[1]
print(np.min(thresh))
cv2.imshow('thresh', thresh)
cv2.waitKey()
cv2.destroyAllWindows()

kernel = np.ones((20,20),np.uint8)
closing = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
cv2.imshow('closing', closing)
cv2.waitKey()
cv2.destroyAllWindows()

thresh = closing

# Find contours
cnts = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
cnts = cnts[0] if len(cnts) == 2 else cnts[1]
c = max(cnts, key=cv2.contourArea)

cv2.fillPoly(image, pts =[c], color=(255, 0, 0))


# Draw dots onto image
cv2.drawContours(image, [c], -1, (36, 255, 12), 2)

cv2.imshow('thresh', thresh)
cv2.imshow('image', image)
cv2.waitKey()