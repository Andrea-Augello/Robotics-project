
import cv2 
import imutils 
from imutils.object_detection import non_max_suppression
import numpy as np

   
# Initializing the HOG person 
# detector 
hog = cv2.HOGDescriptor() 
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector()) 
   
# Reading the Image 
image = cv2.imread('test_images/img2.jpeg') 
   
# Resizing the Image 
image = imutils.resize(image,width=min(400, image.shape[1])) 
   
# Detecting all the regions in the  
# Image that has a pedestrians inside it 
#(regions, _) = hog.detectMultiScale(image, winStride=(4, 4), padding=(4, 4), scale=1.05) 

# detect people in the image
(rects, weights) = hog.detectMultiScale(image, winStride=(4, 4),padding=(8, 8), scale=1.05)
   

# apply non-maxima suppression to the bounding boxes using a
# fairly large overlap threshold to try to maintain overlapping
# boxes that are still people
rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)

# draw the final bounding boxes
for (xA, yA, xB, yB) in pick:
	cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)
  
# Showing the output Image 
cv2.imshow("Image", image) 
cv2.waitKey(0) 
   
cv2.destroyAllWindows() 