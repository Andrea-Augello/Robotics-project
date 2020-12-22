import numpy as np
from imutils.object_detection import non_max_suppression
import rospy
import argparse
import cv2
import os
import time

 # TODO: Make as class 
def extract_boxes_confidences_classids(outputs, confidence, width, height):
    boxes = []
    confidences = []
    classIDs = []

    for output in outputs:
        for detection in output:            
            # Extract the scores, classid, and the confidence of the prediction
            scores = detection[5:]
            classID = np.argmax(scores)
            conf = scores[classID]
            
            # Consider only the predictions that are above the confidence threshold
            if conf > confidence:
                # Scale the bounding box back to the size of the image
                box = detection[0:4] * np.array([width, height, width, height])
                centerX, centerY, w, h = box.astype('int')

                # Use the center coordinates, width and height to get the coordinates of the top left corner
                x = int(centerX - (w / 2))
                y = int(centerY - (h / 2))

                boxes.append([x, y, int(w), int(h)])
                confidences.append(float(conf))
                classIDs.append(classID)

    return boxes, confidences, classIDs


def draw_bounding_boxes(image, boxes, confidences, classIDs, idxs, colors,labels):
    if len(idxs) > 0:
        for i in idxs.flatten():
            # extract bounding box coordinates
            x, y = boxes[i][0], boxes[i][1]
            w, h = boxes[i][2], boxes[i][3]

            # draw the bounding box and label on the image
            color = [int(c) for c in colors[classIDs[i]]]
            cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
            text = "{}: {:.4f}".format(labels[classIDs[i]], confidences[i])
            cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    return image

def make_prediction(net, layer_names, labels, image, confidence, threshold):
    height, width = image.shape[:2]
    
    # Create a blob and pass it through the model
    blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
    net.setInput(blob)
    outputs = net.forward(layer_names)

    # Extract bounding boxes, confidences and classIDs
    boxes, confidences, classIDs = extract_boxes_confidences_classids(outputs, confidence, width, height)

    # Apply Non-Max Suppression
    idxs = cv2.dnn.NMSBoxes(boxes, confidences, confidence, threshold)

    return boxes, confidences, classIDs, idxs

def get_rois(image_list):
    paths = os.path.split(os.path.realpath(__file__))
    BASE_DIR = paths[0]
    
    global collage 
    collage = np.hstack(image_list)
    cv2.imwrite(BASE_DIR + "/test_images/collage.png", collage)

    #Path to label file
    labels=BASE_DIR+'/model/coco.names' 

    #Path to configuration file
    config=BASE_DIR+'/model/yolov3-tiny.cfg'

    #Path to model weights
    weights=BASE_DIR+'/model/yolov3-tiny.weights'

    #Minimum confidence for a box to be detected
    confidence=0.3

    #Threshold for Non-Max Suppression
    threshold=0.3

    # Get the labels
    labels = open(labels).read().strip().split('\n')

    # Create a list of colors for the labels
    colors = np.random.randint(0, 255, size=(len(labels), 3), dtype='uint8')

    # Load weights using OpenCV
    net = cv2.dnn.readNetFromDarknet(config, weights)

    # Get the ouput layer names
    layer_names = net.getLayerNames()
    layer_names = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    image = cv2.imread(BASE_DIR + "/test_images/collage.png")

    samples = []
    for i in range(2*len(image_list)-1):
        sample = image[0:480, 320*i:320*i+640]
        samples.append(sample)

    roi=[]
    for i in range(2*len(image_list)-1):
        image=samples[i]
        boxes, confidences, classIDs, idxs = make_prediction(net, layer_names, labels, image, confidence, threshold)

        # Test boxes
        image2 = draw_bounding_boxes(image.copy(), boxes, confidences, classIDs, idxs, colors,labels)
        cv2.imshow('YOLO Object Detection', image2)
        cv2.waitKey(0)

        # The class ID for a person is 0.
        # If a person is detected returs the corresponding bounding box
        counter=0
        for j in boxes:
            classID=classIDs[counter]
            if(classID==0):
                j[0]=j[0]+i*320
                roi.append(j)
            counter=counter+1
    # pick = non_max_suppression( np.array([[x, y, x + w, y + h] for (x, y, w, h) in roi]), probs=None, overlapThresh=0.50)
    return roi # np.array([[x1, y1, x2-x1, y2-y1] for (x1, y1, x2, y2) in pick])

#Command line test
if __name__ == '__main__':
   print('Main')
