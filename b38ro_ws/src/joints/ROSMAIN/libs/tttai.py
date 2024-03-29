def tttai(board):

    def minimax(board, depth, is_maximizing):
        if win(1, board):
            return -10
        elif win(2, board):
            return 10
        elif 0 not in board:
            return 0

        if is_maximizing:
            best_score = -float('inf')
            for i in range(9):
                if board[i] == 0:
                    board[i] = 2  # AI move
                    score = minimax(board, depth + 1, False)
                    board[i] = 0  # Undo move
                    best_score = max(score, best_score)
            return best_score
        else:
            best_score = float('inf')
            for i in range(9):
                if board[i] == 0:
                    board[i] = 1  # Human move
                    score = minimax(board, depth + 1, True)
                    board[i] = 0  # Undo move
                    best_score = min(score, best_score)
            return best_score

    best_score = -float('inf')
    move = -1
    for i in range(9):
        if board[i] == 0:
            board[i] = 2  # AI move
            if win(2, board):
                board[i] = 0  # Undo move for next iteration
                return i  # Immediate return on winning move
            score = minimax(board, 0, False)
            board[i] = 0  # Undo move
            if score > best_score:
                best_score = score
                move = i

    return move
def haswin(player, board):
    win_conditions = [(0, 1, 2), (3, 4, 5), (6, 7, 8),(0, 3, 6), (1, 4, 7), (2, 5, 8),(0, 4, 8), (2, 4, 6)]
    return any(all(board[i] == player for i in condition) for condition in win_conditions)

