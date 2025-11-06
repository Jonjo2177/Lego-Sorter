import cv2
import numpy as np
import serial.tools.list_ports

#-----------------------------------
#       Connection to Arduino
#-----------------------------------
ports = serial.tools.list_ports.comports() 
serialInst = serial.Serial() #instance 
portsLists = [] #formats port to ports list

for one in ports:
    portsLists.append(str(one)) #finds all ports
    print(f"Device: {one.device}, Description: {one.description}")
    print(str(one)) #displays all ports available

com = input("Select Com Port for Arduino #: ")

#search and set up for port
for i in range(len(portsLists)):
    if portsLists[i].startswith(str(com)):
        use = str(com)
        print(use)


serialInst.baudrate = 9600
serialInst.port = use
serialInst.open()
#-----------------------------------
#       CV Lego Detection 
#-----------------------------------

cap = cv2.VideoCapture('Green Piece.mov')  # 0 = default webcam
#cap = cv2.VideoCapture(0)  # 0 = default webcam

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # --- Color Ranges ---
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([140, 255, 255])
    lower_green = np.array([35, 40, 40])
    upper_green = np.array([85, 255, 255])

    # --- Masks ---
    mask_red = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1),
                              cv2.inRange(hsv, lower_red2, upper_red2))
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    # --- Detection Function ---
    def detect_and_label(mask, color_name, draw_color, serialInst, last_color, x):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x+w, y+h), draw_color, 2)
                cv2.putText(frame, color_name, (x, y-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, draw_color, 2)

            if last_color != color_name & x < 1:
                x = 1
                print(x)
                serialInst.write((color_name + "\n").encode('utf-8'))
                print('color_name:', color_name)
                last_color = color_name
                print('func_last_color:', last_color)
                x = x + 1
                print(x)
                return last_color

            elif x == 1:
                x = 0
                return last_color

    # --- Detect Each Color ---

    last_color = None
    counter = 0
    #print('1_last_color:', last_color)
    last_color = detect_and_label( mask_red, "Red", (0,0,255), serialInst, last_color, counter)
    #print('R_last_color:', last_color)

    last_color = detect_and_label( mask_blue, "Blue", (255,0,0), serialInst, last_color, counter)
    #print('B_last_color:', last_color)

    last_color = detect_and_label( mask_green, "Green", (0,255,0), serialInst, last_color, counter)    
    #print('G_last_color:', last_color)


    # --- Show Windows ---
    cv2.imshow("Detected Colors", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
exit()
