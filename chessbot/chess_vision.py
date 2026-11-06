#!/usr/bin/env python3
"""
chess_vision.py  -  camera -> human move detection -> Stockfish -> bestmove

How it works
------------
1. No markers needed. The camera is fixed, so the board is found ONCE by
   clicking its 4 corners (or --auto to detect it from the empty board's grid).
   Corners are saved to board_corners.json. When the starting position is set
   up, the view is rotated automatically so white is at the bottom.
   Every frame is then warped to a top-down 800x800 image, 100 px per square.
2. Each square is classified as EMPTY / WHITE / BLACK by comparing colour
   features against references learned from the starting position.
   (We never identify piece TYPES - python-chess remembers those.)
3. When the human presses SPACE, every legal move is tried in software and the
   one whose resulting board best matches the camera is chosen. This handles
   captures, castling, en passant, promotion and a couple of noisy squares.
4. Stockfish picks the reply. It is printed as "bestmove e7e5" and written to
   bestmove.txt. FEN is written to current.fen after every move.
5. Once the reply has been made on the real board, press SPACE and the board
   is re-checked against the expected position.

Keys in the preview window:  SPACE = I've finished my move / board is fixed
                             q / ESC = quit

Run:
    python chess_vision.py --camera http://192.168.1.5:8080/video --robot black
    python chess_vision.py --camera 0              # USB webcam
    python chess_vision.py --recalibrate           # camera/board moved: find board again
"""
import argparse
import json
import os
import sys
import time

import cv2
import numpy as np
import chess
import chess.engine
import chess.pgn

# ----------------------------------------------------------------- CONFIG ---
DEFAULT_CAMERA = "http://192.168.1.5:8080/video"  # IP Webcam URL, or "0" for USB
DEFAULT_STOCKFISH = "stockfish"                   # path to the Stockfish binary
ENGINE_TIME = 0.5          # seconds of thinking per robot move
WARP_SIZE = 800
SQ = WARP_SIZE // 8
CORNERS_FILE = "board_corners.json"
FEN_FILE = "current.fen"
WINDOW = "board (SPACE = done, q = quit)"
MIN_MATCH = 61             # squares (out of 64) that must agree to accept a move

EMPTY, WHITE, BLACK = 0, 1, 2


# ----------------------------------------------------------------- CAMERA ---
class Camera:
    def __init__(self, src):
        self.cap = cv2.VideoCapture(int(src) if str(src).isdigit() else src)
        if not self.cap.isOpened():
            sys.exit(f"Could not open camera: {src}")
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def read(self):
        ok, frame = self.cap.read()
        return frame if ok else None

    def fresh(self, flush=6, average=3):
        """Drop buffered (stale) frames, then average a few for less noise."""
        for _ in range(flush):
            self.cap.grab()
        frames = [f.astype(np.float32) for f in (self.read() for _ in range(average))
                  if f is not None]
        if not frames:
            raise RuntimeError("Camera returned no frames")
        return (sum(frames) / len(frames)).astype(np.uint8)


# ------------------------------------------------------- BOARD LOCATION ---
def order_clockwise(pts):
    """Put the 4 corners in clockwise screen order. A mirrored (anticlockwise)
    order can't be fixed by rotating, so it's corrected here."""
    pts = np.float32(pts)
    x, y = pts[:, 0], pts[:, 1]
    area = np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)
    return pts if area > 0 else pts[::-1].copy()


def find_corners_auto(frame):
    """Find the board's outer corners from its 7x7 grid of inner corners.
    Only works on an EMPTY board (pieces hide the grid)."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ok, pts = cv2.findChessboardCornersSB(gray, (7, 7))
    if not ok:
        ok, pts = cv2.findChessboardCorners(
            gray, (7, 7), flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            return None
    # Fit the ideal grid to the detected points, then extrapolate the outer edge.
    grid = np.float32([[c, r] for r in range(1, 8) for c in range(1, 8)])
    h, _ = cv2.findHomography(grid, pts.reshape(-1, 2))
    outer = np.float32([[[0, 0]], [[8, 0]], [[8, 8]], [[0, 8]]])
    return order_clockwise(cv2.perspectiveTransform(outer, h).reshape(4, 2))


def click_corners(frame):
    """Fallback: click the 4 corners of the playing area, going around the board."""
    pts = []
    win = "Click the 4 board corners, going around the edge (ESC = cancel)"

    def on_mouse(event, x, y, *_):
        if event == cv2.EVENT_LBUTTONDOWN and len(pts) < 4:
            pts.append((x, y))

    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    while len(pts) < 4:
        disp = frame.copy()
        for p in pts:
            cv2.circle(disp, p, 6, (0, 0, 255), -1)
        cv2.imshow(win, disp)
        if cv2.waitKey(20) & 0xFF == 27:
            sys.exit("Calibration cancelled")
    cv2.destroyWindow(win)
    return order_clockwise(pts)


class BoardLocator:
    """The camera is fixed, so the board corners are found once and saved."""

    def __init__(self, cam, recalibrate=False, auto=False):
        if os.path.exists(CORNERS_FILE) and not recalibrate:
            with open(CORNERS_FILE) as fh:
                self.corners = np.float32(json.load(fh))
            print(f"Using saved board corners ({CORNERS_FILE}). "
                  "Run with --recalibrate if the camera or board moved.")
            return
        self.corners = None
        if auto:
            input("Remove all pieces from the board so it can be detected, then press Enter...")
            self.corners = find_corners_auto(cam.fresh())
            if self.corners is None:
                print("Couldn't detect the board automatically - click its 4 corners instead.")
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
        return cv2.warpPerspective(frame, m, (WARP_SIZE, WARP_SIZE))

    def orient(self, frame):
        """With the starting position set up, find which way round the board is.
        1. For each of the 4 rotations, calibrate on the starting position and count
           how many squares then read wrong. A sideways (90 degree) view puts the
           pieces in the wrong rows, so it reads badly and gets rejected.
        2. That leaves 'white at bottom' vs 'white at top'. Pick the one where the
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


def square_patch(img, sq):
    """Centre half of a square (avoids neighbours' pieces leaning in)."""
    x0 = chess.square_file(sq) * SQ
    y0 = (7 - chess.square_rank(sq)) * SQ
    m = SQ // 4
    return img[y0 + m:y0 + SQ - m, x0 + m:x0 + SQ - m]


def features(patch):
    lab = cv2.cvtColor(patch, cv2.COLOR_BGR2LAB).reshape(-1, 3).astype(np.float32)
    gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    return np.array([*lab.mean(0), lab[:, 0].std(), edges.mean()], dtype=np.float32)


class SquareClassifier:
    """Nearest-centroid classifier, one set of centroids per square colour."""

    def calibrate(self, warped, board):
        feats = {sq: features(square_patch(warped, sq)) for sq in chess.SQUARES}
        self.scale = np.stack(list(feats.values())).std(0) + 1e-3
        self.centroids = {}
        for light in (True, False):
            for cls in (EMPTY, WHITE, BLACK):
                group = [feats[sq] for sq in chess.SQUARES
                         if is_light(sq) == light and expected_state(board, sq) == cls]
                self.centroids[(light, cls)] = np.mean(group, 0)

    def classify(self, warped):
        out = {}
        for sq in chess.SQUARES:
            f = features(square_patch(warped, sq))
            light = is_light(sq)
            dist = {cls: np.linalg.norm((f - self.centroids[(light, cls)]) / self.scale)
                    for cls in (EMPTY, WHITE, BLACK)}
            out[sq] = min(dist, key=dist.get)
        return out

    def update(self, warped, board, rate=0.2):
        """Drift the references toward a board we're confident about (lighting changes)."""
        for light in (True, False):
            for cls in (EMPTY, WHITE, BLACK):
                group = [features(square_patch(warped, sq)) for sq in chess.SQUARES
                         if is_light(sq) == light and expected_state(board, sq) == cls]
                if group:
                    key = (light, cls)
                    self.centroids[key] = (1 - rate) * self.centroids[key] + rate * np.mean(group, 0)


def mismatches(board, observed):
    return [sq for sq in chess.SQUARES if observed[sq] != expected_state(board, sq)]


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
def draw_overlay(warped, observed, board):
    vis = warped.copy()
    for i in range(9):
        cv2.line(vis, (i * SQ, 0), (i * SQ, WARP_SIZE), (0, 200, 0), 1)
        cv2.line(vis, (0, i * SQ), (WARP_SIZE, i * SQ), (0, 200, 0), 1)
    for sq, state in observed.items():
        f, r = chess.square_file(sq), chess.square_rank(sq)
        centre = (f * SQ + SQ // 2, (7 - r) * SQ + SQ // 2)
        if state == WHITE:
            cv2.circle(vis, centre, SQ // 6, (255, 255, 255), -1)
            cv2.circle(vis, centre, SQ // 6, (0, 0, 0), 2)
        elif state == BLACK:
            cv2.circle(vis, centre, SQ // 6, (0, 0, 0), -1)
            cv2.circle(vis, centre, SQ // 6, (255, 255, 255), 2)
        if board is not None and state != expected_state(board, sq):  # changed vs known position
            cv2.rectangle(vis, (f * SQ + 3, (7 - r) * SQ + 3),
                          ((f + 1) * SQ - 3, (8 - r) * SQ - 3), (0, 0, 255), 3)
    return vis


def wait_for_space(cam, locator, clf, board, prompt):
    """Live preview until SPACE. Returns (warped, observed) or (None, None) on quit."""
    print(prompt)
    while True:
        frame = cam.read()
        if frame is None:
            continue
        warped = locator.warp(frame)
        cv2.imshow(WINDOW,
                   draw_overlay(warped, clf.classify(warped), board))
        key = cv2.waitKey(1) & 0xFF
        if key == ord(" "):
            warped = locator.warp(cam.fresh())
            return warped, clf.classify(warped)
        if key in (ord("q"), 27):
            return None, None


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
    ap.add_argument("--recalibrate", action="store_true",
                    help="find the board again (camera or board moved)")
    ap.add_argument("--auto", action="store_true",
                    help="auto-detect the board (empty board) instead of clicking corners")
    args = ap.parse_args()

    robot_color = chess.WHITE if args.robot == "white" else chess.BLACK
    cam = Camera(args.camera)
    locator = BoardLocator(cam, args.recalibrate, args.auto)
    board = chess.Board()
    clf = SquareClassifier()

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)   # resizable preview window
    while True:
        input("Put ALL 32 pieces in the STARTING position (lighter colour = white),\n"
              "keep hands clear, then press Enter here...")
        frame = cam.fresh()
        locator.orient(frame)          # work out which way round the board is
        warped = locator.warp(frame)
        clf.calibrate(warped, board)
        observed = clf.classify(warped)
        bad = mismatches(board, observed)
        cv2.imshow(WINDOW, draw_overlay(warped, observed, board))
        cv2.waitKey(1)
        if not bad:
            print("Calibration OK - all 64 squares read correctly.")
        else:
            print(f"Note: {len(bad)} square(s) misread at start: "
                  f"{[chess.square_name(s) for s in bad]}. Continuing anyway.")
        break

    engine = chess.engine.SimpleEngine.popen_uci(args.stockfish)
    engine.configure({"Skill Level": max(0, min(20, args.skill))})
    save_fen(board)

    try:
        while not board.is_game_over():
            if board.turn == robot_color:
                mv = engine.play(board, chess.engine.Limit(time=ENGINE_TIME)).move
                print(f"\nbestmove {mv.uci()}")
                with open("bestmove.txt", "w") as fh:
                    fh.write(mv.uci() + "\n")
                print(f"   ({board.san(mv)})")
                board.push(mv)
                save_fen(board)
                # Wait until the move has been made on the real board, then check it.
                while True:
                    warped, observed = wait_for_space(
                        cam, locator, clf, board,
                        f"   Make {mv.uci()} on the board, then press SPACE.")
                    if warped is None:
                        return
                    bad = mismatches(board, observed)
                    if not bad:
                        clf.update(warped, board)
                        break
                    print(f"   Board doesn't match yet: {[chess.square_name(s) for s in bad]}")
            else:
                warped, observed = wait_for_space(cam, locator, clf, board,
                                                  "\nYour move - press SPACE when done.")
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
                clf.update(warped, board)
                save_fen(board)

        print(f"\nGame over: {board.result()} ({board.outcome().termination.name})")
    finally:
        engine.quit()
        cv2.destroyAllWindows()
        with open("game.pgn", "w") as fh:
            print(chess.pgn.Game.from_board(board), file=fh)
        print("Game saved to game.pgn")


if __name__ == "__main__":
    main()
