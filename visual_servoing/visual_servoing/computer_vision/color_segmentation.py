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
    """Helper to display an image in a window. Press any key to close."""
    cv2.imshow("image", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def cd_color_segmentation(img, template):
    
    # convert to hsv
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # mask ranges for orange color
    lower_orange = np.array([0, 100, 80])
    upper_orange = np.array([25, 255, 255])

    # upper_orange = np.array([20,255,200])
    # lower_orange = np.array([5,60,40])

    # cv2.inRange returns a binary image: 255 where pixel is in range, 0 otherwise
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)   # remove small noise
    mask = cv2.dilate(mask, kernel, iterations=2)   # fill gaps, grow back

    
    # outline of a connected white region in the binary mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        return ((0, 0), (0, 0))

    # find the best cone-like contour
    best_contour = None
    best_score = 0

    for contour in contours:
        area = cv2.contourArea(contour)

        # Ignore tiny contours — these are noise
        if area < 50:
            continue

        # Get the bounding rectangle
        x, y, w, h = cv2.boundingRect(contour)
        if w == 0 or h == 0:
            continue

        # Aspect ratio check: cones are typically taller than wide
        # h/w > 0.5 allows for wide cones at close range
        # h/w < 5.0 rejects very thin vertical lines
        aspect_ratio = float(h) / float(w)
         # if aspect_ratio < 0.5 or aspect_ratio > 5.0:
           #  continue

        # # USE FOR LINE INTEAD OF CONE
        if aspect_ratio < 0.1 or aspect_ratio > 10.0:
             continue

        # ratio of contour area to its convex hull area.
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        if hull_area == 0:
            continue
        solidity = float(area) / float(hull_area)

        # Score: we want large, solid, cone-shaped contours.
        # Multiplying area by solidity rewards big solid shapes.
        score = area * solidity

        if score > best_score:
            best_score = score
            best_contour = contour

    if best_contour is None:
        return ((0, 0), (0, 0))

    # Return the bounding box
    x, y, w, h = cv2.boundingRect(best_contour)
    bounding_box = ((x, y), (x + w, y + h))

    return bounding_box
