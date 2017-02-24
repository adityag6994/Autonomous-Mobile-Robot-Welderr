# import the necessary packages
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
from skimage.morphology import skeletonize, skeletonize_3d
from skimage.data import binary_blobs
import cv2
import math
from skimage.morphology import medial_axis, skeletonize, skeletonize_3d
from skimage.filters import threshold_otsu
from scipy import ndimage
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
from scipy.spatial import KDTree
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
from scipy.spatial import KDTree
 
#please give the path to your image 

# load the games image
image0 = cv2.imread("crack.jpg")
cv2.imshow("Image", image0)

###convert to grey scal
image1 = cv2.cvtColor( image0, cv2.COLOR_RGB2GRAY )

###thresholding
ret,image2 = cv2.threshold(image1,200,255,cv2.THRESH_BINARY)

# remove noise
image3 = cv2.GaussianBlur(image2,(3,3),0)

# convolute with proper kernels
image4 = cv2.Laplacian(image3,cv2.CV_64F)

###skeletisation
element = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
done = False
size = np.size(image4)
skel = np.zeros(image4.shape,np.uint8)
eroded = cv2.erode(image4,element)
temp = cv2.dilate(eroded,element)
temp = cv2.subtract(image4,temp)
image5 = eroded.copy()
thresh = threshold_otsu(image5)
data = image5 < thresh
skel, distance = medial_axis(data, return_distance=True)
skeleton = medial_axis(data).astype(np.uint8)
print skeleton[0].size

###Nearest neibhour (we have 16233 points now)
point_x = []
point_y = []
point = 0
height, width= image5.shape
it = np.nditer(image4, flags=['multi_index'])

# Create a black image
while not it.finished:
	if it[0] > 0:	
		point_x.append(it.multi_index[0])		
		point_y.append(it.multi_index[1])
		temp = it.multi_index	
		point = point + 1
	it.iternext()
	

#arrange them in incresing order wrt x
p_xy = np.column_stack((np.asarray(point_y),np.asarray(point_x)))
copy_p_xy = np.copy(p_xy)
 
p_xy = p_xy[p_xy[:, 0].argsort()]
current_x = -1
current_y = -1
path = []
for i in range(1,len(p_xy)):
	#if already on that y-coordinate
	if current_y == p_xy[i][0]:
		continue
	else:
		current_y = p_xy[i][0]
		path.append(p_xy[i])

import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


list1 = [0]
list2 = [1]
yList = [[l[i] for i in list1] for l in path]
xList = [[l[i] for i in list2] for l in path]

plt.plot(xList, yList, 'b*',linewidth=1.0)
plt.axis([0, 400, 0, 980])
plt.show()



