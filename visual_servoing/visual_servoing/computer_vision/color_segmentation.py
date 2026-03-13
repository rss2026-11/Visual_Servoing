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

import cv2
import numpy as np

def image_print(img):
    cv2.imshow("image", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


import cv2
import numpy as np

def image_print(img):
    cv2.imshow("image", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def find_best_cone(contours, strict=True):
    best_contour = None
    best_score = 0

    for c in contours:
        area = cv2.contourArea(c)
        if area < (30 if strict else 15):
            continue

        x, y, w, h = cv2.boundingRect(c)
        if w == 0 or h == 0:
            continue

        aspect_ratio = float(h) / float(w)
        if strict:
            if aspect_ratio < 0.5 or aspect_ratio > 5.0:
                continue
        else:
            if aspect_ratio < 0.3 or aspect_ratio > 8.0:
                continue

        hull = cv2.convexHull(c)
        hull_area = cv2.contourArea(hull)
        if hull_area == 0:
            continue
        solidity = float(area) / float(hull_area)

        if strict:
            score = area * solidity
        else:
            score = area

        if score > best_score:
            best_score = score
            best_contour = c

    return best_contour


def cd_color_segmentation(img, template):
    bounding_box = ((0, 0), (0, 0))

    hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_orange = np.array([0, 80, 80])
    upper_orange = np.array([25, 255, 255])

    mask = cv2.inRange(hsv_image, lower_orange, upper_orange)

    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_contour = find_best_cone(contours, strict=True)

    if best_contour is None:
        lower_dark = np.array([0, 40, 30])
        upper_dark = np.array([28, 255, 255])
        mask_dark = cv2.inRange(hsv_image, lower_dark, upper_dark)
        mask_dark = cv2.erode(mask_dark, kernel, iterations=1)
        mask_dark = cv2.dilate(mask_dark, kernel, iterations=2)
        contours_dark, _ = cv2.findContours(mask_dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_contour = find_best_cone(contours_dark, strict=False)

    if best_contour is None:
        return ((0, 0), (0, 0))

    x, y, w, h = cv2.boundingRect(best_contour)
    bounding_box = ((x, y), (x + w, y + h))

    return bounding_box



# def cd_color_segmentation(img, template):
#     """
#     Implement the cone detection using color segmentation algorithm
#     Input:
#         img: np.3darray; the input image with a cone to be detected. BGR.
#         template: Not required, but can optionally be used to automate setting hue filter values.
#     Return:
#         bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
#             (x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
#     """
#     ########## YOUR CODE STARTS HERE ##########

#     bounding_box = ((0, 0), (0, 0))

#     # Convert to HSV
#     hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

#     # Two ranges to catch orange that wraps around hue=0
#     lower_orange1 = np.array([0, 120, 50])
#     upper_orange1 = np.array([15, 255, 255])
#     lower_orange2 = np.array([15, 120, 50])
#     upper_orange2 = np.array([30, 255, 255])

#     mask1 = cv2.inRange(hsv_image, lower_orange1, upper_orange1)
#     mask2 = cv2.inRange(hsv_image, lower_orange2, upper_orange2)
#     mask = mask1 | mask2

#     # Morphological cleanup: close small gaps first, then remove noise
#     kernel = np.ones((5, 5), np.uint8)
#     mask = cv2.dilate(mask, kernel, iterations=2)
#     mask = cv2.erode(mask, kernel, iterations=2)

#     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#     if len(contours) == 0:
#         return ((0, 0), (0, 0))

#     # Filter out tiny contours (noise)
#     min_area = 50
#     contours = [c for c in contours if cv2.contourArea(c) > min_area]

#     if len(contours) == 0:
#         return ((0, 0), (0, 0))

#     largest = max(contours, key=cv2.contourArea)

#     x, y, w, h = cv2.boundingRect(largest)
#     bounding_box = ((x, y), (x + w, y + h))

#     return bounding_box






    # bounding_box = ((0, 0), (0, 0))

    # # converting image from BGR to HSV
    # hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # # defining lower and upper ranges for orange
    # lower_orange = np.array([5, 100, 20])
    # upper_orange = np.array([25, 255, 255])

    # # seeing if the object inside parameters
    # mask = cv2.inRange(hsv_image, lower_orange, upper_orange)

    # # cleaning and dilating data 
    # kernel = np.ones((5,5), np.uint8)
    # mask = cv2.erode(mask, kernel, iterations=1)
    # mask = cv2.dilate(mask, kernel, iterations=1)  

    # contours, hierarchy  = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # if len(contours) == 0:
    #     return ((0, 0), (0, 0))
    
    # largest = max(contours, key=cv2.contourArea)

    # x, y, w, h = cv2.boundingRect(largest)
    # x2 = x + w
    # y2 = y + h

    # bounding_box = ((x, y), (x2, y2))

    # ########### YOUR CODE ENDS HERE ###########

    # # Return bounding box
    # return bounding_box
