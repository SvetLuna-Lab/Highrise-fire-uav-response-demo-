from __future__ import annotations
from typing import List, Tuple
import heapq

def astar(start: Tuple[int, int], goal: Tuple[int, int], blocked, wind_cost_fn, W: int, H: int) -> List[Tuple[int, int]]:
    """
    A* on a 4-neighborhood grid with:
    - 'blocked(x,y) -> bool'
    - 'wind_cost_fn(x,y) -> non-negative extra cost' (penalize high wind cells)
    """
    sx, sy = start; gx, gy = goal
    def h(x, y): return abs(x - gx) + abs(y - gy)
    def neighbors(x, y):
        for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
            nx, ny = x + dx, y + dy
            if 0 <= nx < W and 0 <= ny < H and not blocked(nx, ny):
                yield nx, ny

    openq = [(0 + h(sx, sy), 0, (sx, sy), None)]
    came = {}
    bestg = { (sx, sy): 0 }

    while openq:
        f, g, node, parent = heapq.heappop(openq)
        if node in came:  # visited
            continue
        came[node] = parent
        if node == (gx, gy):
            # reconstruct
            path = [node]
            cur = node
            while came[cur] is not None:
                cur = came[cur]
                path.append(cur)
            return list(reversed(path))
        x, y = node
        for nx, ny in neighbors(x, y):
            cost = g + 1.0 + wind_cost_fn(nx, ny)
            if (nx, ny) not in bestg or cost < bestg[(nx, ny)]:
                bestg[(nx, ny)] = cost
                heapq.heappush(openq, (cost + h(nx, ny), cost, (nx, ny), node))
    return []  # no path
