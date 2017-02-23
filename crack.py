# import the necessary packages
import numpy as np
import cv2
import matplotlib.pyplot as plt
 
# load the games image
image0 = cv2.imread("crack.jpg")
#print image0[(1)]
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

###KNN (we have 16233 points now)
point_x = []
point_y = []
point = 0
print image5.shape
#print type(image5)
height, width= image5.shape
#if(image4[0:height, 0:width//4] > 200):
#	point = point + 1
#print height, width
#print image5[(1)]
it = np.nditer(image5, flags=['multi_index'])

#getting points on line
#2270 points

# Create a black image
img = np.zeros((980, 980,3), np.uint8)
temp=(0,0)
while not it.finished:
	#it[0] value of pixel
	#it.multi_index[0]/[1] x/y coordinate of image
	
	if it[0] > 0:	
		point_x.append(it.multi_index[0])		
		point_y.append(it.multi_index[1])
		temp = it.multi_index	
		point = point + 1
#	if point > 1000:
#		break			
	it.iternext()
	

#arrange them in incresing order wrt x
p_xy = np.column_stack((np.asarray(point_x),np.asarray(point_y)))
copy_p_xy = np.copy(p_xy)
 
#iterate going in increasing order of x, and select value of y such that its nearest point in list
for 




#cv2.line(img,(0,527),(10,822),(255,0,0),5)
cv2.line(img,(370,966),(375,976),(255,0,0),5)

print p_xy[1:10]
#np.sort(p_xy.view('i8,i8'), order=['f1'], axis=0).view(np.int)
#print p_xy
#keep on going right while finding nearest neibhour with constrin in x, maybe y

print point, len(point_x), len(point_y), len(p_xy)

cv2.imshow("Image", image5)
cv2.imshow("Image1", img)
cv2.waitKey(100000)


