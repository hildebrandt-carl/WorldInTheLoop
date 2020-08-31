import cv2
import numpy as np
import time
import argparse
import matplotlib.pyplot as plt
from matplotlib.colors import hsv_to_rgb
from matplotlib import colors
from scipy import ndimage

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors



img = cv2.imread('./images/gate.png')

scale_percent = 75
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)
img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA) 

rbg_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img_gray = cv2.cvtColor(rbg_img, cv2.COLOR_BGR2GRAY)
image = cv2.GaussianBlur(img_gray, (5, 5), 0)
thresh = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 2)
contours, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnt = contours

contour_list = []
for contour in contours:
    approx = cv2.approxPolyDP(contour,0.01*cv2.arcLength(contour,True),True)
    area = cv2.contourArea(contour)
    # Filter based on length and area
    if (area > 25000):
        print(area)
        contour_list.append(contour)

cv2.drawContours(rbg_img, contour_list,  -1, (255,0,0), 2)

###################################################

# plt.subplot(1, 4, 1)
# plt.title("Light Color")
# plt.imshow(hsv_to_rgb(do_square))
# plt.xticks([])
# plt.yticks([])

# plt.subplot(1, 4, 2)
# plt.title("Dark Color")
# plt.imshow(hsv_to_rgb(lo_square))
# plt.xticks([])
# plt.yticks([])

fig1 = plt.figure()
plt.subplot(1, 1, 1)
plt.title("Original Image")
plt.imshow(rbg_img)
plt.xticks([])
plt.yticks([])

plt.show()
