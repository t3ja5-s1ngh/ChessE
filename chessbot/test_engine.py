"""
Step 3 test: play against Stockfish by typing moves (no camera needed).
Type moves like  e2e4  or  e4  or  Nf3.   Type  quit  to stop.
Shows the board, the FEN, and the gantry commands the robot would send.
"""
import chess
import chess.engine

STOCKFISH = "stockfish.exe"   # in the same folder as this script
SKILL = 5                     # 0 (weak) .. 20 (full strength)
THINK_TIME = 0.5              # seconds per robot move


def gantry_commands(board, mv):
    """Same logic as chess_vision.py: break a move into simple physical moves."""
    n = chess.square_name
    cmds = []
    letter = board.piece_at(mv.from_square).symbol().upper()
    if board.is_en_passant(mv):
        victim = chess.square(chess.square_file(mv.to_square), chess.square_rank(mv.from_square))
        cmds.append(f"MOVE {n(victim)} GRAVEYARD P")
    elif board.is_capture(mv):
        cmds.append(f"MOVE {n(mv.to_square)} GRAVEYARD {board.piece_at(mv.to_square).symbol().upper()}")
    cmds.append(f"MOVE {n(mv.from_square)} {n(mv.to_square)} {letter}")
    if board.is_castling(mv):
        rank = chess.square_rank(mv.from_square)
        rf, rt = (7, 5) if chess.square_file(mv.to_square) == 6 else (0, 3)
        cmds.append(f"MOVE {n(chess.square(rf, rank))} {n(chess.square(rt, rank))} R")
    if mv.promotion:
        cmds.append(f"PROMOTE {n(mv.to_square)} {chess.piece_symbol(mv.promotion).upper()}")
    return cmds


board = chess.Board()
engine = chess.engine.SimpleEngine.popen_uci(STOCKFISH)
engine.configure({"Skill Level": SKILL})
print("You are WHITE. Robot is BLACK.\n")

try:
    while not board.is_game_over():
        print(board, "\n")
        if board.turn == chess.WHITE:
            text = input("Your move: ").strip()
            if text == "quit":
                break
            try:
                mv = board.parse_san(text)          # e4, Nf3, O-O ...
            except ValueError:
                try:
                    mv = chess.Move.from_uci(text)  # e2e4 ...
                except ValueError:
                    mv = None
            if mv is None or mv not in board.legal_moves:
                print("Illegal or unreadable move, try again.\n")
                continue
            board.push(mv)
        else:
            mv = engine.play(board, chess.engine.Limit(time=THINK_TIME)).move
            print(f"Robot plays {board.san(mv)} ({mv.uci()})")
            for c in gantry_commands(board, mv):
                print("   gantry ->", c)
            board.push(mv)
        print("FEN:", board.fen(), "\n")
    print("Result:", board.result())
finally:
    engine.quit()
