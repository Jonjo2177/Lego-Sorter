import cv2

# Load image
img = cv2.imread("Assignment1_Dinosaur copy.jpg")

# Show image in a window
cv2.imshow("My Image", img)
cv2.waitKey(0)   # waits until you press a key
cv2.destroyAllWindows