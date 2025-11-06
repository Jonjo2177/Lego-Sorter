# pip install pyserial
import cv2
import numpy as np
import time
import serial
import serial.tools.list_ports

#-----------------------------------
#       Connection to Arduino
#-----------------------------------

#---Helper to list all available ports---
def choose_port(): 
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports found. Plug in your Arduino and try again.")

    print("Available serial ports:")
    for idx, p in enumerate(ports):
        # p.device is the actual path, e.g. /dev/tty.usbmodem1101 (macOS)
        print(f"[{idx}]  Device: {p.device} | Description: {p.description}")

    while True:
        sel = input("Select the index of your Arduino port: ").strip()
        if sel.isdigit() and 0 <= int(sel) < len(ports):
            return ports[int(sel)].device
        print("Invalid selection. Please enter a valid index (e.g., 0, 1, 2...).")

#---Helper to open serial port and use specific device---
def open_serial(port, baud=9600, timeout=0.2):
    ser = serial.Serial()
    ser.baudrate = baud
    ser.port = port
    ser.timeout = timeout          # non-blocking reads with small wait
    ser.write_timeout = 0.5
    ser.open()

    # Give Arduino time to reset after opening the port (common on UNO/Nano).
    time.sleep(2.0)

    # Clear any boot noise from the buffer.
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser

# ---Helper to read arduino serial prints---
def read_arduino(ser):
    """
    Non-blocking read: drains all available lines from the input buffer.
    Prints any non-empty decoded lines.
    """
    # If nothing buffered yet, return quickly
    if ser.in_waiting == 0:
        return

    # Read all available lines currently buffered
    while ser.in_waiting > 0:
        raw = ser.readline()  # respects ser.timeout; returns b'' if no full line
        if not raw:
            # No full line (no '\n') yet; stop to avoid busy loop
            break
        line = raw.decode('utf-8', errors='ignore').strip()
        if line:
            print(line)


#-----------------------------------
#       CV Lego Detection 
#-----------------------------------

#cap = cv2.VideoCapture('Green Piece.mov')  # 0 = default webcam
cap = cv2.VideoCapture(0)  # 0 = default webcam

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
    def detect_and_label(mask, color_name, draw_color, ser, last_color):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x+w, y+h), draw_color, 2)
                cv2.putText(frame, color_name, (x, y-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, draw_color, 2)
            if last_color != color_name:
                arduino_send(last_color, color_name)
        
        return last_color
    
    def arduino_send (last_color, color_name, x = 1):

        while x < 2:

            if last_color != color_name:

                # Many Arduino sketches read lines; send newline so it can parse the command.
                to_send = (color_name + "\n").encode("utf-8")
                ser.write(to_send)
                ser.flush()  # push bytes out promptly

                print('color_name:', color_name)
                last_color = color_name
                print('func_last_color:', last_color)
            break

def main():
    port = choose_port()
    ser = open_serial(port)

    try:
        while True:
            #command = input("Arduino Command (Blue/Green/Red/exit): ").strip()

            # --- Detect Each Color ---
            last_color = None
            print('1_last_color:', last_color)
            last_color = detect_and_label( mask_red, "Red", (0,0,255), serialInst, last_color)
            #print('R_last_color:', last_color)

            last_color = detect_and_label( mask_blue, "Blue", (255,0,0), serialInst, last_color)
            #print('B_last_color:', last_color)

            last_color = detect_and_label( mask_green, "Green", (0,255,0), serialInst, last_color)    
            #print('G_last_color:', last_color)


            # --- Show Windows ---
            cv2.imshow("Detected Colors", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Give the Arduino a brief moment to respond, then drain all replies.
            time.sleep(0.05)
            read_arduino(ser)
            
                
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        try:
            ser.close()
            cap.release()
            cv2.destroyAllWindows()
            exit()
        except Exception:
            pass



if __name__ == "__main__":
    main()
