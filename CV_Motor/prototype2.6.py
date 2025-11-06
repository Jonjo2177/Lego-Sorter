# pip install pyserial opencv-python numpy
import cv2
import numpy as np
import time
import serial
import serial.tools.list_ports

# ---------------------------
# Serial helpers
# ---------------------------
def choose_port(): 
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports found. Plug in your Arduino and try again.")

    print("Available serial ports:")
    for idx, p in enumerate(ports):
        print(f"[{idx}]  Device: {p.device} | Description: {p.description}")

    while True:
        sel = input("Select the index of your Arduino port: ").strip()
        if sel.isdigit() and 0 <= int(sel) < len(ports):
            return ports[int(sel)].device
        print("Invalid selection. Please enter a valid index (e.g., 0, 1, 2...).")

def open_serial(port, baud=9600, timeout=0.2):
    ser = serial.Serial(port=port, baudrate=baud, timeout=timeout, write_timeout=0.5)
    time.sleep(2.0)  # allow Arduino reset
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser

def read_arduino(ser):
    """Drain and print any complete lines available from Arduino."""
    if ser.in_waiting == 0:
        return
    while ser.in_waiting > 0:
        raw = ser.readline()
        if not raw:
            break
        line = raw.decode("utf-8", errors="ignore").strip()
        if line:
            print(f"[Arduino] {line}")

def arduino_send(ser, color_name):
    """Send a single line to Arduino."""
    to_send = (color_name + "\n").encode("utf-8")
    ser.write(to_send)
    ser.flush()

# ---------------------------
# Vision helpers
# ---------------------------
def largest_contour(mask, min_area=500):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    # pick the largest contour to reduce spam
    cnt = max(contours, key=cv2.contourArea)
    if cv2.contourArea(cnt) < min_area:
        return None
    return cnt

def draw_box_and_label(frame, cnt, label, bgr_color):
    x, y, w, h = cv2.boundingRect(cnt)
    cv2.rectangle(frame, (x, y), (x+w, y+h), bgr_color, 2)
    cv2.putText(frame, label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, bgr_color, 2)

# ---------------------------
# Main
# ---------------------------
def main():
    # ---- Serial setup ----
    port = choose_port()
    ser = open_serial(port)

    # ---- Video setup ----
    cap = cv2.VideoCapture(0)  # webcam
    #cap = cv2.VideoCapture('Green Piece.mov')  # file

    if not cap.isOpened():
        print("Error: could not open video source.")
        ser.close()
        return

    # HSV color ranges
    lower_red1 = np.array([0,   120, 70])
    upper_red1 = np.array([10,  255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([140, 255, 255])
    lower_green = np.array([35,  40,  40])
    upper_green = np.array([85,  255, 255])

    last_sent_color = None
    last_send_ts = 0.0
    cooldown_ms = 300  # avoid spamming the Arduino

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                # End of file or camera error
                break

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            mask_red = cv2.bitwise_or(
                cv2.inRange(hsv, lower_red1, upper_red1),
                cv2.inRange(hsv, lower_red2, upper_red2)
            )
            mask_blue  = cv2.inRange(hsv, lower_blue,  upper_blue)
            mask_green = cv2.inRange(hsv, lower_green, upper_green)

            # Optional: small morphological cleanup to reduce noise
            kernel = np.ones((3,3), np.uint8)
            mask_red   = cv2.morphologyEx(mask_red,   cv2.MORPH_OPEN, kernel, iterations=1)
            mask_blue  = cv2.morphologyEx(mask_blue,  cv2.MORPH_OPEN, kernel, iterations=1)
            mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel, iterations=1)

            # Detect one main blob per color (largest), draw, and maybe send
            detections = []  # (color_name, bgr, contour)
            cnt = largest_contour(mask_red)
            if cnt is not None:
                detections.append(("Red",   (0,0,255),   cnt))
            cnt = largest_contour(mask_blue)
            if cnt is not None:
                detections.append(("Blue",  (255,0,0),   cnt))
            cnt = largest_contour(mask_green)
            if cnt is not None:
                detections.append(("Green", (0,255,0),   cnt))

            # Draw everything we detected
            for cname, bgr, cnt in detections:
                draw_box_and_label(frame, cnt, cname, bgr)

            # Decide what to send (simple priority: first detected in list)
            if detections:
                color_to_send = detections[0][0]
                now = time.time() * 1000.0
                if color_to_send != last_sent_color and (now - last_send_ts) >= cooldown_ms:
                    arduino_send(ser, color_to_send)
                    print(f"Sent: {color_to_send}")
                    last_sent_color = color_to_send
                    last_send_ts = now

            cv2.imshow("Detected Colors", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Read any replies from Arduino (non-blocking-ish)
            read_arduino(ser)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        try:
            ser.close()
        except Exception:
            pass
        try:
            cap.release()
            cv2.destroyAllWindows()
        except Exception:
            pass

if __name__ == "__main__":
    main()
