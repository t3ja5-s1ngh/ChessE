"""
board_tester.py  -  type paths by hand and send them to the gantry over USB.
(Same as gantry_sender.py; this copy is just for manual testing.)

Talks to complete_hub_movement_with_servo.ino exactly like a person typing in
the Serial Monitor: waits for "Enter START cell", sends the start cell, waits
for "Enter PATH", sends the path, then waits for "Done" before the next command.

IMPORTANT: close the Arduino IDE's Serial Monitor first - only one program can
use the port at a time. Hub must be on y9 when the Arduino starts.

Find the port: Arduino IDE -> Tools -> Port (e.g. COM5), or Device Manager.

Ways to use it
--------------
1. Paste commands yourself (testing the gantry with no chess set):
       python board_tester.py --port COM5
   then type a start cell and a path on one line, e.g.
       b8 {b91, x91, x61, c60}
   or on two lines (start line, then path line), exactly as the planner prints.

2. Send the plan chess_vision.py saved last:
       python board_tester.py --port COM5 --file gantry_commands.txt

3. Plan and send a chess move from the starting position (or any FEN):
       python board_tester.py --port COM5 --move e2e4
       python board_tester.py --port COM5 --move b8c6 --fen "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq - 0 1"

4. No Arduino connected? Add --dry-run to just print what would be sent.
"""
import argparse
import re
import sys
import time

try:
    import serial  # pip install pyserial
except ImportError:
    serial = None

BAUD = 115200
MOVE_TIMEOUT = 180      # seconds allowed for one command (travel + servo + pause)
ERROR_WORDS = ("Bad entry", "Please enter", "Try again", "diagonal",
               "Too many", "needs at least")

CELL = r"[yhgfedcbaxYHGFEDCBAX][0-9]"


class GantryError(Exception):
    pass


class Gantry:
    """One USB connection to the gantry Arduino."""

    def __init__(self, port=None, dry_run=False, conn=None, echo=True):
        self.dry_run = dry_run
        self.echo = echo
        self.ser = conn
        if dry_run or conn is not None:
            return
        if serial is None:
            sys.exit("pyserial is missing:  python -m pip install pyserial")
        try:
            self.ser = serial.Serial(port, BAUD, timeout=0.2)
        except serial.SerialException as e:
            sys.exit(f"Could not open {port}: {e}\n"
                     "Is the Serial Monitor still open? Is it the right COM port?")
        print(f"Connected to {port}. Waiting for the Arduino to restart...")
        # Opening the port restarts most Arduinos; wait for the first prompt.
        self._wait_for("Enter START", timeout=10, quiet_ok=True)

    # -- low level ---------------------------------------------------------
    def _readline(self):
        raw = self.ser.readline()
        if not raw:
            return None
        line = raw.decode(errors="ignore").strip()
        if line and self.echo:
            print(f"      arduino: {line}")
        return line

    def _wait_for(self, text, timeout, quiet_ok=False):
        """Read lines until one contains text. Raises on firmware errors."""
        end = time.time() + timeout
        while time.time() < end:
            line = self._readline()
            if line is None:
                continue
            if text in line:
                return line
            if any(w in line for w in ERROR_WORDS):
                raise GantryError(f"Arduino rejected the command: {line}")
        if quiet_ok:
            return None
        raise GantryError(f"Timed out waiting for '{text}' from the Arduino")

    def _send(self, text):
        if self.echo:
            print(f"   -> {text}")
        self.ser.write((text + "\n").encode())

    # -- one command ---------------------------------------------------------
    def run(self, start, path):
        """Send one command (start cell + path) and wait until the hub is home."""
        start, path = start.strip(), path.strip()
        if self.dry_run:
            print(f"   [dry run] start {start}   path {path}")
            return
        self._send(start)
        self._wait_for("Enter PATH", timeout=MOVE_TIMEOUT)   # hub is at the start cell
        self._send(path)
        self._wait_for("Done", timeout=MOVE_TIMEOUT)          # piece moved, hub home
        self._wait_for("Enter START", timeout=10)             # ready for the next one

    def run_steps(self, steps):
        """steps = list of planner Steps (gantry_planner.Step)."""
        for i, st in enumerate(steps, 1):
            print(f"   gantry {i}/{len(steps)}: {st.label.strip()}")
            self.run(st.start, st.path_str)

    def close(self):
        if self.ser is not None and not self.dry_run:
            self.ser.close()


# ------------------------------------------------------------- PARSING ---
def parse_commands(text):
    """Find (start, path) pairs in text: 'b8 {b91, x61, c60}' on one or two lines."""
    tokens = re.findall(rf"\{{[^}}]*\}}|\b{CELL}\b", text)
    cmds, start = [], None
    for t in tokens:
        if t.startswith("{"):
            if start is None:
                raise ValueError(f"Path {t} has no start cell before it")
            cmds.append((start.lower(), t.lower()))
            start = None
        else:
            start = t
    if start is not None:
        raise ValueError(f"Start cell {start} has no path after it")
    return cmds


# ---------------------------------------------------------------- MAIN ---
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="e.g. COM5 (Windows) or /dev/ttyUSB0")
    ap.add_argument("--file", help="send all commands in this file (e.g. gantry_commands.txt)")
    ap.add_argument("--move", help="plan + send this chess move, e.g. e2e4")
    ap.add_argument("--fen", default=None, help="position for --move (default: start position)")
    ap.add_argument("--dry-run", action="store_true", help="don't talk to the Arduino, just print")
    args = ap.parse_args()

    if not args.port and not args.dry_run:
        args.port = input("Arduino port (Arduino IDE -> Tools -> Port), e.g. COM5: ").strip()

    gantry = Gantry(args.port, dry_run=args.dry_run)
    try:
        if args.move:
            import chess
            from gantry_planner import GantryPlanner, print_plan
            board = chess.Board(args.fen) if args.fen else chess.Board()
            mv = chess.Move.from_uci(args.move)
            if mv not in board.legal_moves:
                sys.exit(f"{args.move} is not legal in that position.")
            steps, notes = GantryPlanner().plan(board, mv)
            print_plan(steps, notes)
            gantry.run_steps(steps)
        elif args.file:
            with open(args.file) as fh:
                cmds = parse_commands(fh.read())
            if not cmds:
                sys.exit(f"No commands found in {args.file}")
            for i, (s, p) in enumerate(cmds, 1):
                print(f"   gantry {i}/{len(cmds)}")
                gantry.run(s, p)
        else:
            print("Paste a command: start cell + path, e.g.   b8 {b91, x91, x61, c60}")
            print("(two lines works too). Type q to quit.\n")
            pending = ""
            while True:
                line = input("> " if not pending else "path> ").strip()
                if line.lower() in ("q", "quit", "exit"):
                    break
                pending = (pending + " " + line).strip()
                if "{" in pending and "}" not in pending:
                    continue                      # path still being typed
                try:
                    cmds = parse_commands(pending)
                except ValueError as e:
                    if re.fullmatch(CELL, pending):
                        continue                  # got a start cell, wait for the path
                    print(f"   {e}")
                    pending = ""
                    continue
                pending = ""
                for s, p in cmds:
                    gantry.run(s, p)
                    print("   done.\n")
        print("All commands finished.")
    except GantryError as e:
        print(f"\nERROR: {e}\nThe Arduino may still be waiting for input. "
              "Press its reset button (with the hub on y9) before trying again.")
    finally:
        gantry.close()


if __name__ == "__main__":
    main()
