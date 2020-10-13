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


file_names = ['./images/gate_sphinx_overlay.png',
              './images/gate_unity_overlay.png' ,
              './images/gate_real_overlay.png',
              './images/large_gate_real_overlay.png',
              './images/large_gate_real_overlay2.png']

for name in file_names:

    img = cv2.imread(name)

    scale_percent = 100
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA) 

    rbg_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    hsv_img = cv2.cvtColor(rbg_img, cv2.COLOR_RGB2HSV)

    # Colors we want to find
    dark_color = (0, 100, 110)
    light_color = (255, 255, 255) 

    # Create the selected colors for display
    lo_square = np.full((10, 10, 3), dark_color, dtype=np.uint8) / 255.0
    do_square = np.full((10, 10, 3), light_color, dtype=np.uint8) / 255.0

    # Mask out everything but black
    mask = cv2.inRange(hsv_img, dark_color, light_color)
    result = cv2.bitwise_and(rbg_img, rbg_img, mask=mask)

    ################PLOT THE HSV SCATTER PLOT#################
    if scale_percent <= 25:
        h, s, v = cv2.split(hsv_img)
        fig = plt.figure()
        axis = fig.add_subplot(1, 1, 1, projection="3d")

        pixel_colors = rbg_img.reshape((np.shape(rbg_img)[0]*np.shape(rbg_img)[1], 3))
        norm = colors.Normalize(vmin=-1.,vmax=1.)
        norm.autoscale(pixel_colors)
        pixel_colors = norm(pixel_colors).tolist()

        axis.scatter(h.flatten(), s.flatten(), v.flatten(), facecolors=pixel_colors, marker=".")
        axis.set_xlabel("Hue")
        axis.set_ylabel("Saturation")
        axis.set_zlabel("Value")
    ##################################################

    fig1 = plt.figure()
    plt.subplot(2, 1, 1)
    plt.title("Original Image")
    plt.imshow(rbg_img)
    plt.xticks([])
    plt.yticks([])

    plt.subplot(2, 1, 2)
    plt.title("Gate Position")
    plt.imshow(result)
    plt.xticks([])
    plt.yticks([])

plt.show()
