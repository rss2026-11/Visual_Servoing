import cv2
import numpy as np

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################


def image_print(img):
    """
    Helper function to print out images, for debugging. Pass them in as a list.
    Press any key to continue.
    """
    cv2.imshow("image", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def cd_color_segmentation(img, template):
    """
    Implement the cone detection using color segmentation algorithm
    Input:
        img: np.3darray; the input image with a cone to be detected. BGR.
        template: Not required, but can optionally be used to automate setting hue filter values.
    Return:
        bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
            (x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
    """
    ########## YOUR CODE STARTS HERE ##########

    bounding_box = ((0, 0), (0, 0))

    # converting image from BGR to HSV
    hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # defining lower and upper ranges for orange
    lower_orange = np.array([5, 100, 20])
    upper_orange = np.array([25, 255, 255])

    # seeing if the object inside parameters
    mask = cv2.inRange(hsv_image, lower_orange, upper_orange)

    # cleaning and dilating data 
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=1)  

    contours, hierarchy  = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        return ((0, 0), (0, 0))
    
    largest = max(contours, key=cv2.contourArea)

    x, y, w, h = cv2.boundingRect(largest)
    x2 = x + w
    y2 = y + h

    bounding_box = ((x, y), (x2, y2))

    ########### YOUR CODE ENDS HERE ###########

    # Return bounding box
    return bounding_box
