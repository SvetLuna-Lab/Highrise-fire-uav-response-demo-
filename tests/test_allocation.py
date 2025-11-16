from src.allocation import assign_tasks

def test_hungarian_basic():
    # 2 UAVs, 2 fires
    eta = [
        [1.0, 5.0],
        [2.0, 1.0],
    ]
    pairs = assign_tasks(eta)
    assert len(pairs) == 2
    # optimal is (0->0, 1->1)
    assert (0,0) in pairs and (1,1) in pairs
