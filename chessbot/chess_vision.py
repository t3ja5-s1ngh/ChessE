#!/usr/bin/env python3
"""
chess_vision.py  -  camera -> human move detection -> Stockfish -> bestmove
                    -> gantry plan

How it works
------------
1. The camera watches the WHOLE 10x10 workspace: the 8x8 chessboard in the
   middle plus the 1-cell parking ring around it (y column, x column, row 0,
   row 9). You click the 4 OUTER corners of the 10x10 grid once; they are saved
   to workspace_corners.json. When the starting position is set up, the view is
   rotated automatically so white is at the bottom.
   Every frame is warped to a top-down 1000x1000 image, 100 px per cell.
2. Each cell is classified as EMPTY / WHITE / BLACK by comparing colour
   features against references learned from the starting position (parking
   ring empty at start). We never identify piece TYPES - python-chess
   remembers those.
3. When the human presses SPACE, every legal move is tried in software and the
   one whose resulting board best matches the camera is chosen. This handles
   captures, castling, en passant, promotion and a couple of noisy squares.
4. Stockfish picks the reply: printed as "bestmove e7e5", written to
   bestmove.txt. FEN is written to current.fen after every move.
5. gantry_planner.py turns the reply into gantry commands (start cell + path,
   with magnet flags), printed and written to gantry_commands.txt. Which
   parking cells are full comes from the camera.
6. With --port COMx the commands are sent to the Arduino automatically and the
   board + parking ring are checked by the camera as soon as the gantry is done.
   Without --port, make the move yourself (or paste the commands) and press SPACE.

Preview colours: GREEN grid = chessboard, ORANGE grid = parking ring.
Keys in the preview window:  SPACE = I've finished my move / board is fixed
                             q / ESC = quit

Run:
    python chess_vision.py --camera http://172.16.129.68:8080/video --robot black
    python chess_vision.py --port COM5             # gantry moves the robot's pieces
    python chess_vision.py --camera 0              # USB webcam
    python chess_vision.py --recalibrate           # camera/board moved: click corners again
"""
import argparse
import json
import os
import sys
import threading
import time

# Quieter OpenCV/FFmpeg logs, and a read timeout so a dead stream can't hang us.
os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")
os.environ.setdefault("OPENCV_FFMPEG_LOGLEVEL", "-8")
os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rw_timeout;5000000")

import cv2
import numpy as np
import chess
import chess.engine
import chess.pgn

from gantry_planner import (GantryPlanner, PlanError, print_plan,
                            sq_to_xy, xy_name, LANE_CELLS)
from gantry_sender import Gantry, GantryError

# ----------------------------------------------------------------- CONFIG ---
DEFAULT_CAMERA = "http://172.16.129.68:8080/video"  # IP Webcam URL, or "0" for USB
DEFAULT_STOCKFISH = "stockfish"                   # path to the Stockfish binary
ENGINE_TIME = 0.5          # seconds of thinking per robot move
CELLS = 10                 # workspace is 10 x 10 cells
CELL = 100                 # pixels per cell in the flattened image
WARP_SIZE = CELLS * CELL
CORNERS_FILE = "workspace_corners.json"
FEN_FILE = "current.fen"
WINDOW = "board (SPACE = done, q = quit)"
MIN_MATCH = 58             # squares (out of 64) that must agree to accept a move

EMPTY, WHITE, BLACK = 0, 1, 2
BOARD_COLOUR = (0, 200, 0)     # BGR green  - chessboard grid
LANE_COLOUR = (0, 140, 255)    # BGR orange - parking ring grid


# ----------------------------------------------------------------- CAMERA ---
class Camera:
    """Reads the stream continuously in a background thread, so the newest frame
    is always available - even after the program was busy (e.g. while the
    gantry moves). Reconnects automatically if the stream drops."""

    def __init__(self, src):
        self.src = int(src) if str(src).isdigit() else src
        self.cap = self._open()
        if not self.cap.isOpened():
            sys.exit(f"Could not open camera: {src}")
        self.lock = threading.Lock()
        self.frame, self.frame_id = None, 0
        self.last_ok = time.time()
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()
        if not self._wait_frames(1, timeout=10):
            sys.exit(f"Camera opened but sent no frames: {src}")

    def _open(self):
        cap = cv2.VideoCapture(self.src)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _loop(self):
        while self.running:
            ok, f = self.cap.read()
            if ok and f is not None:
                with self.lock:
                    self.frame, self.frame_id = f, self.frame_id + 1
                self.last_ok = time.time()
            elif time.time() - self.last_ok > 3:
                print("   (camera stream lost - reconnecting...)")
                self.cap.release()
                time.sleep(0.5)
                self.cap = self._open()
                self.last_ok = time.time()
            else:
                time.sleep(0.01)

    def _wait_frames(self, n, timeout):
        """Wait until n new frames have arrived. Returns them (or [] on timeout)."""
        with self.lock:
            last = self.frame_id
        frames, end = [], time.time() + timeout
        while len(frames) < n and time.time() < end:
            with self.lock:
                fid, f = self.frame_id, self.frame
            if fid > last:
                frames.append(f.copy())
                last = fid
            else:
                time.sleep(0.01)
        return frames

    def read(self):
        """Newest frame (may be the same as last time)."""
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def fresh(self, average=3, skip=2, timeout=15):
        """Frames captured AFTER this call: skip a couple (may have been in
        flight), then average a few for less noise."""
        frames = self._wait_frames(skip + average, timeout)[skip:]
        if not frames:
            raise RuntimeError("Camera sent no new frames - check the phone / Wi-Fi")
        return (sum(f.astype(np.float32) for f in frames) / len(frames)).astype(np.uint8)

    def release(self):
        self.running = False


# -------------------------------------------------------- CELL GEOMETRY ---
# Planner cells (gx, gy): gx = column y,h,g..a,x = 0..9, gy = row 0..9 as drawn
# on the workspace (row 0 on the WHITE side). In the flattened image white is
# at the bottom, so the drawing appears rotated by 180 degrees.
def cell_origin(cell):
    gx, gy = cell
    return (9 - gx) * CELL, (9 - gy) * CELL      # top-left pixel (x, y)


def cell_patch(img, cell):
    """Centre half of a cell (avoids neighbours' pieces leaning in)."""
    x0, y0 = cell_origin(cell)
    m = CELL // 4
    return img[y0 + m:y0 + CELL - m, x0 + m:x0 + CELL - m]


def square_patch(img, sq):
    return cell_patch(img, sq_to_xy(sq))


# ------------------------------------------------------- SHADOW REMOVAL ---
SHADOW_FIX = True   # set False to switch the lighting correction off


def even_out_light(img):
    """Remove shadows / uneven lighting. The paper is the brightest thing in any
    small area (the corners of every cell are paper, round pieces don't cover
    them), so a local maximum filter estimates how bright the paper is lit at
    each spot. Dividing by it makes a shadowed cell look like a lit one."""
    if not SHADOW_FIX:
        return img
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # "Closing" = max filter then min filter: wipes out dark things smaller than
    # a cell (pieces, grid lines) but keeps the sharp edge of a big shadow.
    k = int(CELL * 1.1) | 1
    paper = cv2.morphologyEx(gray, cv2.MORPH_CLOSE,
                             cv2.getStructuringElement(cv2.MORPH_RECT, (k, k)))
    paper = cv2.GaussianBlur(paper, (0, 0), CELL / 10).astype(np.float32)
    gain = 220.0 / np.maximum(paper, 30.0)
    return np.clip(img.astype(np.float32) * gain[..., None], 0, 255).astype(np.uint8)


# ------------------------------------------------------- BOARD LOCATION ---
def order_clockwise(pts):
    """Put the 4 corners in clockwise screen order. A mirrored (anticlockwise)
    order can't be fixed by rotating, so it's corrected here."""
    pts = np.float32(pts)
    x, y = pts[:, 0], pts[:, 1]
    area = np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)
    return pts if area > 0 else pts[::-1].copy()


def find_corners_auto(frame):
    """Find the workspace's outer corners from the chessboard's 7x7 grid of inner
    corners. Only works on an EMPTY board with real light/dark squares."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ok, pts = cv2.findChessboardCornersSB(gray, (7, 7))
    if not ok:
        ok, pts = cv2.findChessboardCorners(
            gray, (7, 7), flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            return None
    grid = np.float32([[c, r] for r in range(1, 8) for c in range(1, 8)])
    h, _ = cv2.findHomography(grid, pts.reshape(-1, 2))
    # 8x8 board spans 0..8; the 10x10 workspace is one cell further out.
    outer = np.float32([[[-1, -1]], [[9, -1]], [[9, 9]], [[-1, 9]]])
    return order_clockwise(cv2.perspectiveTransform(outer, h).reshape(4, 2))


def overlay_on_camera(frame, corners):
    """Draw the 10x10 grid (green board, orange ring) straight onto the camera view."""
    src = np.float32([[0, 0], [WARP_SIZE, 0], [WARP_SIZE, WARP_SIZE], [0, WARP_SIZE]])
    m = cv2.getPerspectiveTransform(src, np.float32(corners))     # flat -> camera
    def proj(pts):
        return cv2.perspectiveTransform(np.float32(pts).reshape(-1, 1, 2), m) \
                  .reshape(-1, 2).astype(np.int32)
    view = frame.copy()
    tint = view.copy()
    for gy in range(CELLS):
        for gx in range(CELLS):
            if gx in (0, CELLS - 1) or gy in (0, CELLS - 1):
                cv2.fillPoly(tint, [proj([(gx * CELL, gy * CELL), ((gx + 1) * CELL, gy * CELL),
                                          ((gx + 1) * CELL, (gy + 1) * CELL),
                                          (gx * CELL, (gy + 1) * CELL)])], LANE_COLOUR)
    view = cv2.addWeighted(tint, 0.35, view, 0.65, 0)
    for i in range(CELLS + 1):
        cv2.polylines(view, [proj([(i * CELL, 0), (i * CELL, WARP_SIZE)])], False, LANE_COLOUR, 2)
        cv2.polylines(view, [proj([(0, i * CELL), (WARP_SIZE, i * CELL)])], False, LANE_COLOUR, 2)
    for i in range(9):
        cv2.polylines(view, [proj([(CELL + i * CELL, CELL), (CELL + i * CELL, 9 * CELL)])],
                      False, BOARD_COLOUR, 2)
        cv2.polylines(view, [proj([(CELL, CELL + i * CELL), (9 * CELL, CELL + i * CELL)])],
                      False, BOARD_COLOUR, 2)
    return view


def click_corners(frame):
    """Click the 4 OUTER corners of the full 10x10 grid, going around the edge.
    Then the grid is drawn on the camera view: ENTER = looks right, R = click again."""
    win = "Click the 4 OUTER corners of the 10x10 grid, going around (ESC = cancel)"
    pts = []

    def on_mouse(event, x, y, *_):
        if event == cv2.EVENT_LBUTTONDOWN and len(pts) < 4:
            pts.append((x, y))

    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, on_mouse)
    while True:
        pts.clear()
        while len(pts) < 4:
            disp = frame.copy()
            for p in pts:
                cv2.circle(disp, p, 8, (0, 0, 255), -1)
            cv2.imshow(win, disp)
            if cv2.waitKey(20) & 0xFF == 27:
                sys.exit("Calibration cancelled")
        corners = order_clockwise(pts)
        print("Check the grid on the camera view: ENTER = looks right, R = click again.")
        cv2.imshow(win, overlay_on_camera(frame, corners))
        while True:
            key = cv2.waitKey(0) & 0xFF
            if key in (13, 10, 32):          # Enter (or space)
                cv2.destroyWindow(win)
                return corners
            if key in (ord("r"), ord("R"), ord("c")):
                break
            if key == 27:
                sys.exit("Calibration cancelled")


class BoardLocator:
    """The camera is fixed, so the workspace corners are found once and saved."""

    def __init__(self, cam, recalibrate=False, auto=False):
        if os.path.exists(CORNERS_FILE) and not recalibrate:
            with open(CORNERS_FILE) as fh:
                self.corners = np.float32(json.load(fh))
            print(f"Using saved workspace corners ({CORNERS_FILE}). "
                  "Run with --recalibrate if the camera or board moved.")
            return
        self.corners = None
        if auto:
            input("Remove all pieces from the board so it can be detected, then press Enter...")
            self.corners = find_corners_auto(cam.fresh())
            if self.corners is None:
                print("Couldn't detect the board automatically - click the corners instead.")
            else:
                print("Board detected automatically.")
        if self.corners is None:
            self.corners = click_corners(cam.fresh())
        self.save()

    def save(self):
        with open(CORNERS_FILE, "w") as fh:
            json.dump(self.corners.tolist(), fh)

    def warp(self, frame, corners=None):
        c = self.corners if corners is None else corners
        dst = np.float32([[0, 0], [WARP_SIZE, 0], [WARP_SIZE, WARP_SIZE], [0, WARP_SIZE]])
        m = cv2.getPerspectiveTransform(c, dst)
        return even_out_light(cv2.warpPerspective(frame, m, (WARP_SIZE, WARP_SIZE)))

    def orient(self, frame):
        """With the starting position set up, find which way round the board is.
        1. For each of the 4 rotations, calibrate on the starting position and count
           how many squares then read wrong. A sideways view puts the pieces in the
           wrong rows, so it reads badly and gets rejected.
        2. That leaves 'white at bottom' vs 'white at top': pick the one where the
           LIGHTER pieces are at the bottom.
        Returns the number of misread squares for the chosen rotation."""
        def brightness(warped, ranks):
            return np.mean([cv2.cvtColor(square_patch(warped, chess.square(f, r)),
                                         cv2.COLOR_BGR2GRAY).mean()
                            for r in ranks for f in range(8)])

        start = chess.Board()
        options = [np.roll(self.corners, k, axis=0) for k in range(4)]
        errors, light_bottom = [], []
        for c in options:
            w = self.warp(frame, c)
            clf = SquareClassifier()
            clf.calibrate(w, start)
            errors.append(len(mismatches(start, clf.classify(w))))
            light_bottom.append(brightness(w, (0, 1)) - brightness(w, (6, 7)))
        pair = min((0, 1), key=lambda p: errors[p] + errors[p + 2])
        k = pair if light_bottom[pair] > light_bottom[pair + 2] else pair + 2
        self.corners = options[k]
        self.save()
        return errors[k]


# ------------------------------------------------- SQUARE CLASSIFICATION ---
def is_light(sq):
    return (chess.square_file(sq) + chess.square_rank(sq)) % 2 == 1


def expected_state(board, sq):
    piece = board.piece_at(sq)
    if piece is None:
        return EMPTY
    return WHITE if piece.color == chess.WHITE else BLACK


def features(patch):
    lab = cv2.cvtColor(patch, cv2.COLOR_BGR2LAB).reshape(-1, 3).astype(np.float32)
    gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    return np.array([*lab.mean(0), lab[:, 0].std(), edges.mean()], dtype=np.float32)


LANE = "lane"   # reference group for parking cells


class SquareClassifier:
    """Nearest-centroid classifier with reference looks per background:
    light squares, dark squares and parking cells."""

    def calibrate(self, warped, board):
        """board = starting position. The parking ring must be EMPTY."""
        feats = {sq: features(square_patch(warped, sq)) for sq in chess.SQUARES}
        lane_feats = [features(cell_patch(warped, c)) for c in LANE_CELLS]
        self.scale = np.stack(list(feats.values()) + lane_feats).std(0) + 1e-3
        self.centroids = {}
        for light in (True, False):
            for cls in (EMPTY, WHITE, BLACK):
                group = [feats[sq] for sq in chess.SQUARES
                         if is_light(sq) == light and expected_state(board, sq) == cls]
                self.centroids[(light, cls)] = np.mean(group, 0)
        # Parking cells: empty look from the ring itself; a piece looks like the
        # average of that piece colour on light and dark squares.
        self.centroids[(LANE, EMPTY)] = np.mean(lane_feats, 0)
        for cls in (WHITE, BLACK):
            self.centroids[(LANE, cls)] = (self.centroids[(True, cls)] +
                                           self.centroids[(False, cls)]) / 2

    def _nearest(self, f, group):
        dist = {cls: np.linalg.norm((f - self.centroids[(group, cls)]) / self.scale)
                for cls in (EMPTY, WHITE, BLACK)}
        return min(dist, key=dist.get)

    def classify(self, warped):
        """The 64 chess squares: {square: EMPTY / WHITE / BLACK}."""
        return {sq: self._nearest(features(square_patch(warped, sq)), is_light(sq))
                for sq in chess.SQUARES}

    def classify_lanes(self, warped):
        """The 36 parking cells: {(gx, gy): EMPTY / WHITE / BLACK}."""
        return {c: self._nearest(features(cell_patch(warped, c)), LANE) for c in LANE_CELLS}

    def update(self, warped, board, parked=(), rate=0.2):
        """Drift the references toward a board we're confident about (lighting changes)."""
        for light in (True, False):
            for cls in (EMPTY, WHITE, BLACK):
                group = [features(square_patch(warped, sq)) for sq in chess.SQUARES
                         if is_light(sq) == light and expected_state(board, sq) == cls]
                if group:
                    key = (light, cls)
                    self.centroids[key] = (1 - rate) * self.centroids[key] + rate * np.mean(group, 0)
        empty_lanes = [features(cell_patch(warped, c)) for c in LANE_CELLS if c not in parked]
        if empty_lanes:
            key = (LANE, EMPTY)
            self.centroids[key] = (1 - rate) * self.centroids[key] + rate * np.mean(empty_lanes, 0)


def mismatches(board, observed):
    return [sq for sq in chess.SQUARES if observed[sq] != expected_state(board, sq)]


def occupied_lanes(lanes):
    return {c for c, state in lanes.items() if state != EMPTY}


def lane_mismatches(lanes, parked):
    """Parking cells where the camera disagrees with what should be parked."""
    return [c for c in LANE_CELLS if (lanes[c] != EMPTY) != (c in parked)]


# --------------------------------------------------------- MOVE DETECTION ---
def detect_move(board, observed):
    """Returns (move, score, status) where status is 'ok', 'nochange' or 'unclear'."""
    current = 64 - len(mismatches(board, observed))
    scored = []
    for mv in board.legal_moves:
        after = board.copy(stack=False)
        after.push(mv)
        scored.append((64 - len(mismatches(after, observed)), mv))
    if not scored:
        return None, current, "unclear"
    best = max(s for s, _ in scored)
    if current >= best:
        return None, current, "nochange"
    top = [mv for s, mv in scored if s == best]
    # Promotions look identical from above - default to a queen.
    if len(top) > 1 and all(m.promotion for m in top) \
            and len({(m.from_square, m.to_square) for m in top}) == 1:
        top = [m for m in top if m.promotion == chess.QUEEN]
    if best < MIN_MATCH or len(top) != 1:
        return None, best, "unclear"
    return top[0], best, "ok"


# ------------------------------------------------------------- UI HELPERS ---
def _draw_piece(vis, centre, state):
    r = CELL // 6
    if state == WHITE:
        cv2.circle(vis, centre, r, (255, 255, 255), -1)
        cv2.circle(vis, centre, r, (0, 0, 0), 2)
    elif state == BLACK:
        cv2.circle(vis, centre, r, (0, 0, 0), -1)
        cv2.circle(vis, centre, r, (255, 255, 255), 2)


def _box(vis, cell, colour, thick):
    x0, y0 = cell_origin(cell)
    cv2.rectangle(vis, (x0 + 3, y0 + 3), (x0 + CELL - 3, y0 + CELL - 3), colour, thick)


def draw_overlay(warped, observed, board, lanes=None, parked=None):
    """GREEN = chessboard cells, ORANGE = parking cells. Circles = detected pieces.
    RED box = differs from what the program expects."""
    vis = warped.copy()
    # Tint the parking ring orange so the two areas are easy to tell apart.
    tint = vis.copy()
    for c in LANE_CELLS:
        x0, y0 = cell_origin(c)
        cv2.rectangle(tint, (x0, y0), (x0 + CELL, y0 + CELL), LANE_COLOUR, -1)
    vis = cv2.addWeighted(tint, 0.25, vis, 0.75, 0)
    # Grid lines: board green, ring orange
    for c in LANE_CELLS:
        x0, y0 = cell_origin(c)
        cv2.rectangle(vis, (x0, y0), (x0 + CELL, y0 + CELL), LANE_COLOUR, 1)
    b0, b1 = CELL, 9 * CELL
    for i in range(9):
        cv2.line(vis, (b0 + i * CELL, b0), (b0 + i * CELL, b1), BOARD_COLOUR, 2)
        cv2.line(vis, (b0, b0 + i * CELL), (b1, b0 + i * CELL), BOARD_COLOUR, 2)

    for sq, state in observed.items():
        cell = sq_to_xy(sq)
        x0, y0 = cell_origin(cell)
        _draw_piece(vis, (x0 + CELL // 2, y0 + CELL // 2), state)
        if board is not None and state != expected_state(board, sq):
            _box(vis, cell, (0, 0, 255), 3)
    if lanes is not None:
        for c, state in lanes.items():
            x0, y0 = cell_origin(c)
            _draw_piece(vis, (x0 + CELL // 2, y0 + CELL // 2), state)
            if parked is not None and (state != EMPTY) != (c in parked):
                _box(vis, c, (0, 0, 255), 3)
    return vis


def wait_for_space(cam, locator, clf, board, prompt, parked=None, allow_force=False):
    """Live preview until SPACE. Returns (warped, observed, lanes),
    or (None, None, None) on quit. With allow_force, F returns ("force", ...)."""
    print(prompt)
    while True:
        frame = cam.read()
        if frame is None:
            continue
        warped = locator.warp(frame)
        cv2.imshow(WINDOW, draw_overlay(warped, clf.classify(warped), board,
                                        clf.classify_lanes(warped), parked))
        key = cv2.waitKey(1) & 0xFF
        if key == ord(" "):
            warped = locator.warp(cam.fresh())
            return warped, clf.classify(warped), clf.classify_lanes(warped)
        if allow_force and key in (ord("f"), ord("F")):
            return "force", None, None
        if key in (ord("q"), 27):
            return None, None, None


def changed_cells(before, after, parked_before, parked_after):
    """The cells a robot move should have changed: from/to squares, a captured
    piece's square, castling rook squares, and the new parking cell."""
    cells = [chess.square_name(sq) for sq in chess.SQUARES
             if expected_state(before, sq) != expected_state(after, sq)]
    cells += [xy_name(c) for c in set(parked_after) - set(parked_before)]
    return cells


def check_cells(warped, clf, board, parked, cells):
    """Which of `cells` the camera disagrees with (only those are checked,
    so a shadow somewhere else on the board doesn't matter)."""
    observed, lanes = clf.classify(warped), clf.classify_lanes(warped)
    bad = []
    for name in cells:
        if name[0] in "yx" or name[1] in "09":            # parking cell
            cell = next(c for c in LANE_CELLS if xy_name(c) == name)
            if (lanes[cell] != EMPTY) != (cell in parked):
                bad.append(name)
        elif observed[chess.parse_square(name)] != expected_state(board, chess.parse_square(name)):
            bad.append(name)
    return bad


def write_gantry_plan(planner, board, mv, path="gantry_commands.txt"):
    """Plan the gantry moves for mv (board = position BEFORE mv), print them and
    save them, one command per pair of lines: start cell, then path.
    Returns the list of steps, or None if no plan was found."""
    try:
        steps, notes = planner.plan(board, mv)
    except PlanError as e:
        print(f"   GANTRY: no plan found ({e}). Please make this move by hand.")
        with open(path, "w") as fh:
            fh.write("")
        return None
    print(f"   Gantry plan ({len(steps)} command{'s' if len(steps) > 1 else ''}):")
    print_plan(steps, notes)
    with open(path, "w") as fh:
        for st in steps:
            fh.write(st.start + "\n" + st.path_str + "\n")
    return steps


def save_fen(board):
    fen = board.fen()
    with open(FEN_FILE, "w") as fh:
        fh.write(fen + "\n")
    print(f"   FEN: {fen}")


# ------------------------------------------------------------------- MAIN ---
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default=DEFAULT_CAMERA)
    ap.add_argument("--stockfish", default=DEFAULT_STOCKFISH)
    ap.add_argument("--robot", choices=["white", "black"], default="black")
    ap.add_argument("--skill", type=int, default=10, help="Stockfish skill 0-20")
    ap.add_argument("--port", default=None,
                    help="Arduino port (e.g. COM5): send gantry commands automatically")
    ap.add_argument("--recalibrate", action="store_true",
                    help="click the workspace corners again (camera or board moved)")
    ap.add_argument("--auto", action="store_true",
                    help="auto-detect the board (empty, real light/dark squares) instead of clicking")
    args = ap.parse_args()

    robot_color = chess.WHITE if args.robot == "white" else chess.BLACK
    cam = Camera(args.camera)
    locator = BoardLocator(cam, args.recalibrate, args.auto)
    board = chess.Board()
    clf = SquareClassifier()

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)   # resizable preview window
    input("Put ALL 32 pieces in the STARTING position (lighter colour = white),\n"
          "make sure the PARKING RING IS EMPTY, keep hands clear, then press Enter here...")
    frame = cam.fresh()
    locator.orient(frame)          # work out which way round the board is
    warped = locator.warp(frame)
    clf.calibrate(warped, board)
    observed, lanes = clf.classify(warped), clf.classify_lanes(warped)
    bad = [chess.square_name(s) for s in mismatches(board, observed)]
    bad += [xy_name(c) for c in lane_mismatches(lanes, set())]
    cv2.imshow(WINDOW, draw_overlay(warped, observed, board, lanes, set()))
    cv2.waitKey(1)
    if not bad:
        print("Calibration OK - all 100 cells read correctly.")
    else:
        print(f"Note: {len(bad)} cell(s) misread at start: {bad}. Continuing anyway.")

    planner = GantryPlanner()
    gantry = Gantry(args.port) if args.port else None
    engine = chess.engine.SimpleEngine.popen_uci(args.stockfish)
    engine.configure({"Skill Level": max(0, min(20, args.skill))})
    save_fen(board)

    try:
        while not board.is_game_over():
            if board.turn == robot_color:
                # Which parking cells are full: ask the camera.
                warped = locator.warp(cam.fresh())
                planner.parked = occupied_lanes(clf.classify_lanes(warped))

                mv = engine.play(board, chess.engine.Limit(time=ENGINE_TIME)).move
                print(f"\nbestmove {mv.uci()}")
                with open("bestmove.txt", "w") as fh:
                    fh.write(mv.uci() + "\n")
                print(f"   ({board.san(mv)})")
                before, parked_before = board.copy(), set(planner.parked)
                steps = write_gantry_plan(planner, board, mv)   # also updates planner.parked
                board.push(mv)
                save_fen(board)
                key_cells = changed_cells(before, board, parked_before, planner.parked)
                auto_ok = False
                if gantry and steps:
                    try:
                        gantry.run_steps(steps)
                        for attempt in range(2):          # 2nd look covers stream lag
                            time.sleep(1.0 if attempt == 0 else 2.0)
                            warped = locator.warp(cam.fresh())
                            bad = check_cells(warped, clf, board, planner.parked, key_cells)
                            if not bad:
                                break
                        if not bad:
                            print(f"   Gantry done - checked {', '.join(key_cells)}: correct.")
                            auto_ok = True
                        else:
                            print(f"   Gantry done, but these look wrong: {bad}")
                    except GantryError as e:
                        print(f"   GANTRY ERROR: {e}\n   Reset the Arduino (hub on y9), "
                              "then finish the move by hand.")
                # Otherwise: wait until the move has been made, then check the
                # changed cells. F = the board is right, carry on anyway.
                while not auto_ok:
                    warped, observed, lanes = wait_for_space(
                        cam, locator, clf, board,
                        f"   Make {mv.uci()} on the board, then press SPACE "
                        "(or F if the board is right and the camera is wrong).",
                        planner.parked, allow_force=True)
                    if warped is None:
                        return
                    if isinstance(warped, str):            # F pressed
                        print("   OK - carrying on.")
                        break
                    bad = check_cells(warped, clf, board, planner.parked, key_cells)
                    if not bad:
                        break
                    print(f"   Still looks wrong: {bad}. Fix it and press SPACE, "
                          "or press F to carry on anyway.")
            else:
                warped, observed, lanes = wait_for_space(
                    cam, locator, clf, board, "\nYour move - press SPACE when done.",
                    planner.parked)
                if warped is None:
                    return
                mv, score, status = detect_move(board, observed)
                if status == "nochange":
                    print("   No move detected.")
                    continue
                if status == "unclear":
                    print(f"   Couldn't match a legal move (best {score}/64). "
                          "Check the board and press SPACE again.")
                    continue
                print(f"   You played {board.san(mv)} ({mv.uci()}) [{score}/64 squares agree]")
                board.push(mv)
                clf.update(warped, board, planner.parked)
                save_fen(board)

        print(f"\nGame over: {board.result()} ({board.outcome().termination.name})")
    finally:
        engine.quit()
        cam.release()
        if gantry:
            gantry.close()
        cv2.destroyAllWindows()
        with open("game.pgn", "w") as fh:
            print(chess.pgn.Game.from_board(board), file=fh)
        print("Game saved to game.pgn")


if __name__ == "__main__":
    main()
