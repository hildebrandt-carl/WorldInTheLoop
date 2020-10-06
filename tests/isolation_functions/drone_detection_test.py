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


img = cv2.imread('./images/simulationexample.png')
img = cv2.imread('./images/realexample.png')

scale_percent = 100
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)
img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA) 

rbg_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
hsv_img = cv2.cvtColor(rbg_img, cv2.COLOR_RGB2HSV)

# Colors we want to find
dark_black = (0, 0, 0)
light_black = (255, 255, 100)

# Create the selected colors for display
lo_square = np.full((10, 10, 3), dark_black, dtype=np.uint8) / 255.0
do_square = np.full((10, 10, 3), light_black, dtype=np.uint8) / 255.0

# # Create the selection
# topleft = [350, 220]
# bottomright = [480, 270]

# Mask out everything but black
mask = cv2.inRange(hsv_img, dark_black, light_black)
result = cv2.bitwise_and(rbg_img, rbg_img, mask=mask)

# # Isolate certain region of the image
# rectange_mask = np.full(hsv_img.shape, 255, dtype=np.uint8)
# roi_corners = np.array([[(topleft[0], topleft[1]), (bottomright[0], topleft[1]), (bottomright[0], bottomright[1]), (topleft[0], bottomright[1])]], dtype=np.int32)
# cv2.fillPoly(rectange_mask, roi_corners, 0)
# result = cv2.bitwise_or(result, rectange_mask)

# Invert image to make it easier to see
result = cv2.bitwise_not(result, mask=mask)
result = cv2.bitwise_not(result)

# # Create the lines which show the area we have cropped
# scatter_x = np.append(roi_corners[0][:,0], roi_corners[0][0,0])
# scatter_y = np.append(roi_corners[0][:,1], roi_corners[0][0,1])

fig1 = plt.figure()
plt.subplot(1, 2, 1)
plt.title("Original Image")
# plt.plot(scatter_x, scatter_y)
plt.imshow(rbg_img)
plt.xticks([])
plt.yticks([])

plt.subplot(1, 2, 2)
plt.title("Drone Position")
plt.imshow(result)
plt.xticks([])
plt.yticks([])

plt.show()
