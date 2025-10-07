import cv2 as cv

capture = cv.VideoCapture('Blue.MOV')



while True:
    isTrue, frame = capture.read()

    b,g,r = cv.split(frame)

    blue_channel = cv.merge([b, g*0, r*0])

    cv.imshow('Blue_Color_Detection', blue_channel)
    cv.imshow('Blue_Test', frame)

    if cv.waitKey(20) & 0xFF==ord('d'):
        break

capture.release()
cv.destroyAllWindows()

cv.waitKey(0)