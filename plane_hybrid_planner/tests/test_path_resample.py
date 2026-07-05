import numpy as np

from plane_hybrid_planner.path_resample import (
    resample_path_by_arclength,
    simplify_path_optional,
)


def test_resample_preserves_endpoints_and_count():
    path = np.asarray([[0.0, 0.0], [0.2, 0.0], [0.2, 0.8], [1.0, 1.0]])
    result = resample_path_by_arclength(path, 30)
    assert result.shape == (30, 2)
    assert np.allclose(result[0], path[0])
    assert np.allclose(result[-1], path[-1])


def test_simplify_preserves_endpoints():
    path = np.asarray([[0.0, 0.0], [0.01, 0.0], [0.02, 0.0], [1.0, 1.0]])
    result = simplify_path_optional(path, min_dist=0.05)
    assert result.shape[0] == 2
    assert np.allclose(result[[0, -1]], path[[0, -1]])

