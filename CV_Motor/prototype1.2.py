# pip install pyserial
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

def main():
    port = choose_port()
    ser = open_serial(port)

    try:
        while True:
            command = input("Arduino Command (Blue/Green/Red/exit): ").strip()
            if not command:
                continue
            if command.lower() == "exit":
                print("Exiting.")
                break

            # Many Arduino sketches read lines; send newline so it can parse the command.
            to_send = (command + "\n").encode("utf-8")
            ser.write(to_send)
            print(to_send)
            ser.flush()  # push bytes out promptly

            # Give the Arduino a brief moment to respond, then drain all replies.
            time.sleep(0.05)
            read_arduino(ser)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        try:
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
