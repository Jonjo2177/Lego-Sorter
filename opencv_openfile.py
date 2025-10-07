import cv2 as cv
import sys 

# Load image
img = cv.imread(cv.samples.findFile("Assignment1_Dinosaur copy.jpg"))

if img is None:
    sys.exit("Could not read/find image.")

#Shows the image in a window
cv.imshow("Display window", img)
k = cv.waitKey(0) # waits until you press a key

if k == ord("s"): # converts image to png?
    cv.imwrite("Assignment1_Dinosaur copy.png", img)

if k == ord("c"):
    cv.destroyAllWindows