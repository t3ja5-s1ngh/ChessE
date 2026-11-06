"""
Step 2 test: find the chessboard and flatten it to a top-down 8x8 grid.

Keys (in the camera window):
  d = try to DETECT the board automatically (board must be EMPTY)
  c = CLICK the 4 corners yourself (click around the edge, in order)
  s = save the flattened board as board_flat.jpg
  q = quit
"""
import cv2
import numpy as np

URL = "http://172.16.128.114:8080/video"   # <-- same URL as test_camera.py
SIZE = 800                              # flattened board is 800x800 (100 px squares)


def auto_detect(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ok, pts = cv2.findChessboardCornersSB(gray, (7, 7))
    if not ok:
        return None
    # The 49 inner corners -> work out where the 4 outer corners are.
    grid = np.float32([[c, r] for r in range(1, 8) for c in range(1, 8)])
    h, _ = cv2.findHomography(grid, pts.reshape(-1, 2))
    outer = np.float32([[[0, 0]], [[8, 0]], [[8, 8]], [[0, 8]]])
    return cv2.perspectiveTransform(outer, h).reshape(4, 2)


def click_corners(frame):
    pts = []
    win = "Click the 4 board corners, going around the edge"

    def on_mouse(event, x, y, *_):
        if event == cv2.EVENT_LBUTTONDOWN and len(pts) < 4:
            pts.append((x, y))

    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    while len(pts) < 4:
        disp = frame.copy()
        for p in pts:
            cv2.circle(disp, p, 8, (0, 0, 255), -1)
        cv2.imshow(win, disp)
        if cv2.waitKey(20) & 0xFF == 27:   # ESC cancels
            break
    cv2.destroyWindow(win)
    return np.float32(pts) if len(pts) == 4 else None


def flatten(frame, corners):
    dst = np.float32([[0, 0], [SIZE, 0], [SIZE, SIZE], [0, SIZE]])
    m = cv2.getPerspectiveTransform(corners, dst)
    flat = cv2.warpPerspective(frame, m, (SIZE, SIZE))
    for i in range(9):  # draw the 8x8 grid so you can check alignment
        cv2.line(flat, (i * SIZE // 8, 0), (i * SIZE // 8, SIZE), (0, 255, 0), 2)
        cv2.line(flat, (0, i * SIZE // 8), (SIZE, i * SIZE // 8), (0, 255, 0), 2)
    return flat


cap = cv2.VideoCapture(URL)
if not cap.isOpened():
    raise SystemExit("Could not open the camera stream.")

corners = None
print(__doc__)
while True:
    ok, frame = cap.read()
    if not ok:
        continue
    view = frame.copy()
    if corners is not None:
        cv2.polylines(view, [corners.astype(int)], True, (0, 0, 255), 3)
        flat = flatten(frame, corners)
        cv2.imshow("flattened board", flat)
    cv2.imshow("camera (d = detect, c = click corners, s = save, q = quit)", view)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("d"):
        found = auto_detect(frame)
        if found is None:
            print("Board not found. Is it EMPTY and fully in view? Try 'c' to click instead.")
        else:
            corners = found
            print("Board detected automatically!")
    elif key == ord("c"):
        clicked = click_corners(frame)
        if clicked is not None:
            corners = clicked
            print("Corners set.")
    elif key == ord("s") and corners is not None:
        cv2.imwrite("board_flat.jpg", flatten(frame, corners))
        print("Saved board_flat.jpg")
    elif key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
