import cv2
import numpy as np
import copy
import matplotlib.pyplot as plt
from matplotlib.colors import hsv_to_rgb
from matplotlib import colors
from scipy import ndimage

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors


# Load the iamges
overlay_original    = cv2.imread('images/gate_real_overlay.png')
background          = cv2.imread('images/background.png')

# Resize overlay to match the msg image
height, width       = background.shape[:2]
overlay_original    = cv2.resize(overlay_original, (width, height), interpolation = cv2.INTER_CUBIC)

# Convert to RGB
overlay_original    = cv2.cvtColor(overlay_original, cv2.COLOR_BGR2RGB)
overlay_hsv         = cv2.cvtColor(overlay_original, cv2.COLOR_RGB2HSV)
background          = cv2.cvtColor(background, cv2.COLOR_BGR2RGB)

# Colors we want to find
dark_color          = (0, 100, 0)
light_color         = (100, 255, 255)

# Create the selected colors for display
lo_square           = np.full((10, 10, 3), dark_color, dtype=np.uint8) / 255.0
do_square           = np.full((10, 10, 3), light_color, dtype=np.uint8) / 255.0

# Mask out everything but black
mask                = cv2.inRange(overlay_hsv, dark_color, light_color)
overlay             = cv2.bitwise_and(overlay_original, overlay_original, mask=mask)

# Copy the background where the mask is not pure black
output = copy.deepcopy(background)
output[np.where(mask != 0)] = overlay[np.where(mask != 0)]

# Plot the different images
fig1 = plt.figure()
plt.subplot(2, 2, 1)
plt.title("Background Image")
plt.imshow(background)
plt.xticks([])
plt.yticks([])

plt.subplot(2, 2, 2)
plt.title("Overlay Image")
plt.imshow(overlay_original)
plt.xticks([])
plt.yticks([])

plt.subplot(2, 2, 3)
plt.title("Isolated Overlay Image")
plt.imshow(overlay)
plt.xticks([])
plt.yticks([])

plt.subplot(2, 2, 4)
plt.title("Combined Images")
plt.imshow(output)
plt.xticks([])
plt.yticks([])
plt.show()