import pytest
from maze_track import MazeTrack
from maze_solver import MazePlanner

def test_cell_to_world():
    grid = [[2, 1], [0, 3]]
    mt = MazeTrack(grid, cell_size=10.0, origin_x=0, origin_y=0)
    # (0,0) → üst-sol → world: (5, 15)  (çünkü rows=2, r=0 → y = (2-1-0)*10+5 = 15)
    p = mt.cell_to_world(0, 0)
    assert abs(p.x - 5.0) < 0.01
    assert abs(p.y - 15.0) < 0.01

def test_adjacency():
    grid = [[1, 1], [0, 1]]
    mt = MazeTrack(grid, cell_size=10.0)
    adj = mt.get_adjacency()
    assert (0, 1) in adj[(0, 0)]
    assert (1, 1) in adj[(0, 1)]
    assert (0, 0) not in adj.get((1, 0), [])  # (1,0) = 0, adj'de yok

def test_bfs():
    from maze_presets import MAZE_SIMPLE_L
    mt = MazeTrack(MAZE_SIMPLE_L, cell_size=10.0)
    planner = MazePlanner(mt)
    start_pos, start_cell = mt.get_start_position()
    end_pos, end_cell = mt.get_end_position()
    path = planner.bfs(start_cell, end_cell)
    assert len(path) > 0
    assert path[0] == start_cell
    assert path[-1] == end_cell
