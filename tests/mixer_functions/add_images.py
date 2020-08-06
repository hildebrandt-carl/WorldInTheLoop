
import cv2
import matplotlib.pyplot as plt


msg = cv2.imread('msg.png')
overlay = cv2.imread('overlay.png')

msg = cv2.cvtColor(msg, cv2.COLOR_BGR2RGB)
overlay = cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB)

# Resize images
height, width = msg.shape[:2]
overlay = cv2.resize(overlay, (width, height), interpolation = cv2.INTER_CUBIC)

img2gray = cv2.cvtColor(overlay, cv2.COLOR_BGR2GRAY)
ret, mask = cv2.threshold(img2gray, 10, 255, cv2.THRESH_BINARY)
mask_inv = cv2.bitwise_not(mask)
img1_bg = cv2.bitwise_and(msg,msg,mask = mask_inv)
img2_fg = cv2.bitwise_and(overlay,overlay,mask = mask)
output = cv2.add(img1_bg,img2_fg)

fig1 = plt.figure()
plt.subplot(2, 3, 1)
plt.title("Original Image")
plt.imshow(msg)
plt.xticks([])
plt.yticks([])

plt.subplot(2, 3, 2)
plt.title("Overlay Image")
plt.imshow(overlay)
plt.xticks([])
plt.yticks([])

plt.subplot(2, 3, 3)
plt.title("Image Background")
plt.imshow(img1_bg)
plt.xticks([])
plt.yticks([])

plt.subplot(2, 3, 4)
plt.title("Image Forground")
plt.imshow(img2_fg)
plt.xticks([])
plt.yticks([])

plt.subplot(2, 3, 5)
plt.title("Combined Images")
plt.imshow(output)
plt.xticks([])
plt.yticks([])
plt.show()
