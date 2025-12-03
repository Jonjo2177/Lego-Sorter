# pip install pyserial opencv-python numpy matplotlib
import cv2
import numpy as np
import time
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt   # <-- NEW: for graph
import matplotlib.ticker as ticker

plt.gca().xaxis.set_major_locator(ticker.MaxNLocator(integer=True))
plt.gca().yaxis.set_major_locator(ticker.MaxNLocator(integer=True))

# ---- GLOBAL COUNTERS (Python) ----
count_red = 0
count_green = 0
count_blue = 0


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
    global count_red, count_green, count_blue   # <-- we'll modify the globals

    # ---- Serial setup ----
    port = choose_port()
    ser = open_serial(port)

    print("\nConnected to Arduino on", port)

    # -----------------------------------------
    # Ask user for conveyor speed (usCruise)
    # -----------------------------------------
    while True:
        try:
            speed_val0 = int(input("Enter conveyor M0 usCruise (μs, e.g., 1500): "))
            speed_val1 = int(input("Enter conveyor M1 usCruise (μs, e.g., 1500): "))
            break
        except ValueError:
            print("Please enter a valid number.")

    speed_cmd0 = f"SPD0:{speed_val0}\n"
    speed_cmd1 = f"SPD1:{speed_val1}\n"
    ser.write(speed_cmd0.encode('utf-8'))
    print("Sent speed command to M0:", speed_cmd0.strip())
    time.sleep(0.5)
    ser.write(speed_cmd1.encode('utf-8'))
    print("Sent speed command to M1:", speed_cmd1.strip())

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
    lower_green = np.array([45, 100, 90])
    upper_green = np.array([80, 255, 255])


    last_sent_color = None
    last_send_ts = 0.0
    cooldown_ms = 10000  # avoid spamming the Arduino

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

                # Only send every cooldown_ms
                if (now - last_send_ts) >= cooldown_ms:
                    print("it has been", now - last_send_ts, "ms since last send")
                    arduino_send(ser, color_to_send)
                    print(f"Sent: {color_to_send}")
                    last_sent_color = color_to_send
                    last_send_ts = now

                    # ---- UPDATE PYTHON COUNTERS WHEN WE SEND TO ARDUINO ----
                    if color_to_send == "Red":
                        count_red += 1
                    elif color_to_send == "Green":
                        count_green += 1
                    elif color_to_send == "Blue":
                        count_blue += 1

            # ---- DRAW COUNTS ON FRAME (SCOREBOARD) ----
            cv2.putText(frame, f"Red: {count_red}",   (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(frame, f"Green: {count_green}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(frame, f"Blue: {count_blue}",  (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

            cv2.imshow("Detected Colors", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Read any replies from Arduino (non-blocking-ish)
            read_arduino(ser)

            # Console display as well
            print(f"RED: {count_red}, GREEN: {count_green}, BLUE: {count_blue}", end="\r")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        # Cleanup camera & windows
        try:
            ser.close()
        except Exception:
            pass
        try:
            cap.release()
            cv2.destroyAllWindows()
        except Exception:
            pass

# ---- SHOW FINAL GRAPH OF COUNTS (+ REAL COUNTS + % ERROR) ----
        try:
            colors = ["Red", "Green", "Blue"]
            detected_counts = [count_red, count_green, count_blue]
            print("\n\nDetected counts:", dict(zip(colors, detected_counts)))

            # --- Ask user for real (ground truth) counts ---
            print("\nEnter the REAL number of bricks (press Enter to skip a color):")
            try:
                real_red_in = input("Real Red count: ").strip()
                real_green_in = input("Real Green count: ").strip()
                real_blue_in = input("Real Blue count: ").strip()

                # If user leaves blank, use None
                real_counts = []
                for val in (real_red_in, real_green_in, real_blue_in):
                    if val == "":
                        real_counts.append(None)
                    else:
                        real_counts.append(int(val))
            except ValueError:
                print("Invalid input for real counts. Skipping real/ground-truth comparison.")
                real_counts = [None, None, None]

            # --- Print percentage error in console ---
            print("\nPercentage error (Detected vs Real):")
            for color, det, real in zip(colors, detected_counts, real_counts):
                if real is None:
                    print(f"  {color}: N/A (no real count provided)")
                elif real == 0:
                    print(f"  {color}: N/A (real count is 0)")
                else:
                    err = (det - real) / real * 100.0   # signed error
                    abs_err = abs(err)
                    print(f"  {color}: {err:.1f}% (absolute: {abs_err:.1f}%)")

            # --- Build the graph ---
            x = np.arange(len(colors))  # 0,1,2 for R,G,B
            width = 0.35

            plt.figure(figsize=(9,6))
            
            # Detected bars
            det_bars = plt.bar(x - width/2, detected_counts, width, label="Detected")

            # Real bars (only if provided)
            if any(v is not None for v in real_counts):
                real_plot = [v if v is not None else 0 for v in real_counts]
                real_bars = plt.bar(x + width/2, real_plot, width, label="Sorted")
            else:
                real_plot = [0, 0, 0]

            # ---- ADD PERCENT ERROR ON THE GRAPH ----
            for i, (det, real) in enumerate(zip(detected_counts, real_counts)):
                # Skip if user didn't give a real count
                if real is None or real == 0:
                    plt.text(x[i], max(det, 0) + 1,
                            "Real missing",
                            ha='center', va='bottom', fontsize=10, color='gray')
                    continue

                # Compute error
                err = (det - real) / real * 100.0
                sign = "+" if err >= 0 else ""
                txt = f"{sign}{err:.1f}%"

                # Put ABOVE the taller of the two bars
                y_pos = max(det, real) + max(det, real) * 0.05
                plt.text(x[i], y_pos, txt, ha='center', fontsize=12, color='black')

            # Labeling
            plt.xticks(x, colors)
            plt.xlabel("Color")
            plt.ylabel("Count")
            speed_title = input("Speed for Title (Slow, Medium, Fast): ").strip()
            plt.title("LEGO Sorting " + speed_title + ": Detected vs Sorted Counts (with % Error)")
            plt.legend()
            plt.tight_layout()
            plt.show()

        except Exception as e:
            print("Could not display graph:", e)

if __name__ == "__main__":
    main()
