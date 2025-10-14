import cv2
import numpy as np

# ---------- Helpers ----------
def nothing(x): pass

def build_hsv_panel(win="HSV Tuner"):
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 500, 300)
    for name, maxv, init in [
        ("H_min",179,0), ("S_min",255,40), ("V_min",255,40),
        ("H_max",179,179), ("S_max",255,255), ("V_max",255,255),
        ("MinArea",5000,1200), ("MaxArea",200000,120000),
        ("OpenK",21,5), ("CloseK",21,7),
        ("Canny1",500,80), ("Canny2",500,160)
    ]:
        cv2.createTrackbar(name, win, init, maxv, nothing)

def read_hsv_panel(win="HSV Tuner"):
    H_min = cv2.getTrackbarPos("H_min",win)
    S_min = cv2.getTrackbarPos("S_min",win)
    V_min = cv2.getTrackbarPos("V_min",win)
    H_max = cv2.getTrackbarPos("H_max",win)
    S_max = cv2.getTrackbarPos("S_max",win)
    V_max = cv2.getTrackbarPos("V_max",win)
    MinArea = cv2.getTrackbarPos("MinArea",win)
    MaxArea = cv2.getTrackbarPos("MaxArea",win)
    OpenK = cv2.getTrackbarPos("OpenK",win) | 1
    CloseK = cv2.getTrackbarPos("CloseK",win) | 1
    C1 = cv2.getTrackbarPos("Canny1",win)
    C2 = cv2.getTrackbarPos("Canny2",win)
    return (H_min,S_min,V_min,H_max,S_max,V_max,MinArea,MaxArea,OpenK,CloseK,C1,C2)

def nice_box(img, cnt, color=(0,255,0), label="piece"):
    x,y,w,h = cv2.boundingRect(cnt)
    cv2.rectangle(img,(x,y),(x+w,y+h),color,2)
    cv2.putText(img, label, (x, y-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)

def contour_features(cnt):
    area = cv2.contourArea(cnt)
    x,y,w,h = cv2.boundingRect(cnt)
    rect_area = w*h
    aspect = w/h if h>0 else 0
    hull = cv2.convexHull(cnt)
    hull_area = cv2.contourArea(hull)
    solidity = area / hull_area if hull_area>0 else 0
    extent = area / rect_area if rect_area>0 else 0
    return dict(area=area, aspect=aspect, solidity=solidity, extent=extent, w=w, h=h)

# ---------- Main ----------
def main():
    cap = cv2.VideoCapture('Blue.MOV')  # Use 0 for default webcam; on Pi might be 0 or 1
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    build_hsv_panel()

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        # Preprocess
        frame_blur = cv2.GaussianBlur(frame, (5,5), 0)
        hsv = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)

        # Read tuner
        (H_min,S_min,V_min,H_max,S_max,V_max,
         MinArea,MaxArea,OpenK,CloseK,C1,C2) = read_hsv_panel()

        # Color mask (one range). If pieces vary widely in color, run multiple ranges and OR them.
        mask = cv2.inRange(hsv, (H_min,S_min,V_min), (H_max,S_max,V_max))

        # Morphology to clean up
        kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(OpenK,OpenK))
        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(CloseK,CloseK))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)

        # Optional: edge hint for splitting touching bricks
        edges = cv2.Canny(frame_blur, C1, C2)
        edges = cv2.dilate(edges, None, iterations=1)
        # Combine edges with mask to suppress background edges
        mask_edges = cv2.bitwise_and(mask, mask, mask=cv2.bitwise_not(edges))

        # Contours
        cnts, _ = cv2.findContours(mask_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        out = frame.copy()
        count = 0
        for c in cnts:
            f = contour_features(c)
            if f["area"] < MinArea or f["area"] > MaxArea:
                continue

            # Heuristic examples (tune for your bricks/top-down view)
            # - Plates/tiles tend to be flatter (higher extent), long bricks have high aspect
            # - Use these to bucket pieces broadly; refine later
            label = "LEGO"
            if f["aspect"] > 2.2:
                label = "long piece"
            if f["solidity"] < 0.85:
                label = "irregular"
            if f["extent"] > 0.85 and 0.8 < f["aspect"] < 1.25:
                label = "square-ish"

            nice_box(out, c, (0,255,0), f"{label} a={int(f['area'])} ar={f['aspect']:.2f}")
            count += 1

        # Show
        cv2.putText(out, f"Detected pieces: {count}", (20,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 3)
        cv2.putText(out, f"Detected pieces: {count}", (20,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
        cv2.imshow("frame", out)
        cv2.imshow("mask", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
