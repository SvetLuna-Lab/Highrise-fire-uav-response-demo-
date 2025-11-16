from src.planner import astar

def test_astar_finds_path():
    W, H = 10, 10
    def blocked(x, y): return False
    def wcost(x, y): return 0.0
    path = astar((0,0), (9,9), blocked, wcost, W, H)
    assert path[0] == (0,0) and path[-1] == (9,9)
