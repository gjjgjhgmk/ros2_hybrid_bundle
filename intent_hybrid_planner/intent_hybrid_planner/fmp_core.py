import numpy as np


def _pairwise_sqdist(points: np.ndarray, centers: np.ndarray) -> np.ndarray:
    """Compute squared Euclidean distance matrix: (K, N)."""
    diff = points[:, :, None] - centers[:, None, :]
    return np.sum(diff * diff, axis=0).T


def _initialize_centers_along_trajectory(data: np.ndarray, n_clusters: int, length: int) -> np.ndarray:
    """MATLAB-style center initialization from trajectory order."""
    d, _ = data.shape
    length = int(max(1, min(length, data.shape[1])))
    if n_clusters <= 1:
        return data[:, [0]].copy()

    sec_2 = int(np.floor(0.5 * length / (n_clusters + 1)))
    sec = int(np.floor((length - 2 * sec_2) / (n_clusters - 1)))
    sec = max(sec, 1)
    start = max(sec_2 - 1, 0)  # MATLAB 1-based -> Python 0-based.

    idx = np.arange(start, length, sec, dtype=int)
    idx = idx[:n_clusters]
    if idx.size < n_clusters:
        pad = np.linspace(0, length - 1, n_clusters, dtype=int)
        idx = np.unique(np.concatenate([idx, pad]))
        if idx.size < n_clusters:
            tail = np.arange(length - 1, -1, -1, dtype=int)
            merged = np.unique(np.concatenate([idx, tail]))
            idx = merged[:n_clusters]
        else:
            idx = idx[:n_clusters]
    if idx.size < n_clusters:
        idx = np.pad(idx, (0, n_clusters - idx.size), mode="edge")

    centers = np.zeros((d, n_clusters), dtype=float)
    centers[:, : idx.size] = data[:, idx]
    return centers


def gk_clustering(
    data,
    n_clusters,
    m=2.0,
    max_iter=50,
    tol=1e-4,
    max_iter_fcm=30,
    max_iter_gk=30,
):
    """
    MATLAB-equivalent FCM pre-iteration + GK clustering.

    Args:
        data: (D, N)
        n_clusters: cluster count
        m: fuzzifier (kept for compatibility, MATLAB path uses m=2)
        max_iter: legacy alias for GK iterations when max_iter_gk is None
        tol: kept for compatibility; not used in strict MATLAB-equivalent fixed-iter loops
        max_iter_fcm: FCM pre-iteration count
        max_iter_gk: GK iteration count

    Returns:
        C: (D, K)
        inv_covs: (D, D, K)
        U: (K, N)
    """
    _ = m
    _ = tol
    if max_iter_gk is None:
        max_iter_gk = max_iter

    p = np.asarray(data, dtype=float)
    if p.ndim != 2:
        raise ValueError("data must be a 2D array shaped (D, N).")
    d_p, n_p = p.shape
    if n_p <= 0:
        raise ValueError("data must contain at least one sample.")
    if n_clusters <= 0:
        raise ValueError("n_clusters must be positive.")

    n_clusters = int(n_clusters)
    c = _initialize_centers_along_trajectory(p, n_clusters=n_clusters, length=n_p)

    # Initial U from Euclidean distance.
    dist2 = _pairwise_sqdist(p, c)
    dist2 = np.maximum(dist2, 1e-6)
    tmp = 1.0 / dist2
    u = tmp / np.sum(tmp, axis=0, keepdims=True)
    u2 = u * u

    # FCM pre-iteration (fixed count, MATLAB-equivalent).
    for _ in range(int(max(0, max_iter_fcm))):
        row_sum_u2 = np.sum(u2, axis=1)  # (K,)
        row_sum_u2 = np.maximum(row_sum_u2, 1e-12)
        c = (p @ u2.T) / row_sum_u2[None, :]
        dist2 = _pairwise_sqdist(p, c)
        dist2 = np.maximum(dist2, 1e-12)
        tmp = 1.0 / dist2
        u = tmp / np.sum(tmp, axis=0, keepdims=True)
        u2 = u * u

    inv_covs = np.zeros((d_p, d_p, n_clusters), dtype=float)
    md2 = np.maximum(_pairwise_sqdist(p, c), 1e-12)
    a = 1.0 / float(d_p)

    if max_iter_gk <= 0:
        for i in range(n_clusters):
            inv_covs[:, :, i] = np.eye(d_p, dtype=float)
        return c, inv_covs, u

    # GK iterative updates: loop max_iter_gk - 1 times, then one final stored pass.
    for _ in range(int(max(0, max_iter_gk - 1))):
        row_sum_u2 = np.sum(u2, axis=1)
        row_sum_u2 = np.maximum(row_sum_u2, 1e-12)
        c = (p @ u2.T) / row_sum_u2[None, :]

        for i in range(n_clusters):
            dist = p - c[:, i : i + 1]  # (D, N)
            u2d = dist * u2[i : i + 1, :]
            cov_i = (dist @ u2d.T) / row_sum_u2[i]
            cov_i = cov_i + np.eye(d_p, dtype=float) * 1e-9
            det_cov = float(np.linalg.det(cov_i))
            det_cov = max(det_cov, 1e-12)
            n_in_i = (det_cov**a) * np.linalg.inv(cov_i)
            md2[i, :] = np.sum(dist * (n_in_i @ dist), axis=0)

        md2 = np.maximum(md2, 1e-12)
        tmp = 1.0 / md2
        u = tmp / np.sum(tmp, axis=0, keepdims=True)
        u2 = u * u

    # Final pass with stored inv_covs.
    row_sum_u2 = np.sum(u2, axis=1)
    row_sum_u2 = np.maximum(row_sum_u2, 1e-12)
    c = (p @ u2.T) / row_sum_u2[None, :]
    for i in range(n_clusters):
        dist = p - c[:, i : i + 1]
        u2d = dist * u2[i : i + 1, :]
        cov_i = (dist @ u2d.T) / row_sum_u2[i]
        cov_i = cov_i + np.eye(d_p, dtype=float) * 1e-9
        det_cov = float(np.linalg.det(cov_i))
        det_cov = max(det_cov, 1e-12)
        n_in_i = (det_cov**a) * np.linalg.inv(cov_i)
        md2[i, :] = np.sum(dist * (n_in_i @ dist), axis=0)
        inv_covs[:, :, i] = n_in_i

    return c, inv_covs, u


def _build_fmp_data(demo_traj: np.ndarray, time_axis: np.ndarray, alpha: float, demo_dura: float) -> np.ndarray:
    scaled_time = demo_dura * time_axis
    denom = max(float(scaled_time[-1]), 1e-9)
    phase = np.exp(float(alpha) * scaled_time / denom)
    return np.vstack((scaled_time.reshape(1, -1), phase.reshape(1, -1), demo_traj))


def _align_via_times_to_data_axis(
    via_times: np.ndarray,
    raw_time_axis: np.ndarray,
    data_time_axis: np.ndarray,
    demo_dura: float,
) -> np.ndarray:
    """
    Align via_times to Data_test(1,:) scale.

    Heuristic:
    - If via_time range is close to raw time_axis range, treat as unscaled and multiply demo_dura.
    - If via_time range is close to data_time_axis range, treat as already scaled.
    - Otherwise choose the closer range.
    """
    vt = np.asarray(via_times, dtype=float).reshape(-1)
    if vt.size == 0:
        return vt

    raw_max = max(float(np.max(np.abs(raw_time_axis))), 1e-9)
    data_max = max(float(np.max(np.abs(data_time_axis))), 1e-9)
    via_max = float(np.max(np.abs(vt)))

    tol = 0.05
    close_raw = abs(via_max - raw_max) <= tol * raw_max
    close_data = abs(via_max - data_max) <= tol * data_max

    if close_data and (not close_raw):
        return vt
    if close_raw and (not close_data):
        return vt * float(demo_dura)

    err_raw = abs(via_max - raw_max) / raw_max
    err_data = abs(via_max - data_max) / data_max
    if err_data < err_raw:
        return vt
    return vt * float(demo_dura)


def _train_local_regression(
    data_train: np.ndarray,
    u: np.ndarray,
    n_clusters: int,
    location_x: np.ndarray,
    location_y: np.ndarray,
) -> np.ndarray:
    """MATLAB fuzregre_param_train_t1 equivalent."""
    jl = np.argmax(u, axis=0)  # (N,)
    slx = int(location_x.size)
    sly = int(location_y.size)
    p1_u = np.zeros((1 + slx, sly, n_clusters), dtype=float)

    for i in range(n_clusters):
        idx = jl == i
        if not np.any(idx):
            # Keep MATLAB semantics as close as possible while avoiding singular empty solves.
            pick = int(np.argmax(u[i, :]))
            idx = np.zeros(u.shape[1], dtype=bool)
            idx[pick] = True

        zc = data_train[:, idx]
        wi = u[i, idx].reshape(-1)  # (K,)

        x1 = np.column_stack((np.ones(zc.shape[1], dtype=float), zc[location_x, :].T))
        y1 = zc[location_y, :].T
        w = np.diag(wi)

        lhs = x1.T @ w @ x1
        rhs = x1.T @ (wi[:, None] * y1)
        try:
            p1_u[:, :, i] = np.linalg.solve(lhs, rhs)
        except np.linalg.LinAlgError:
            p1_u[:, :, i] = np.linalg.pinv(lhs) @ rhs

    return p1_u


def _compute_md2(data_sel: np.ndarray, centers_sel: np.ndarray, inv_covs_sel: np.ndarray) -> np.ndarray:
    n_clusters = centers_sel.shape[1]
    n_data = data_sel.shape[1]
    md2 = np.zeros((n_clusters, n_data), dtype=float)
    for i in range(n_clusters):
        dist = data_sel - centers_sel[:, i : i + 1]
        md2[i, :] = np.sum(dist * (inv_covs_sel[:, :, i] @ dist), axis=0)
    return np.maximum(md2, 1e-12)


def _predict_from_membership(
    data_test: np.ndarray,
    p1_u: np.ndarray,
    membership: np.ndarray,
    location_x: np.ndarray,
    d_out: int,
) -> np.ndarray:
    n_data = data_test.shape[1]
    x = np.column_stack((np.ones(n_data, dtype=float), data_test[location_x, :].T))
    y = np.zeros((d_out, n_data), dtype=float)
    for j in range(membership.shape[0]):
        y += (x @ p1_u[:, :, j]).T * membership[j : j + 1, :]
    return y


def train_fmp_model(
    demo_traj,
    time_axis,
    N_C=20,
    alpha=0.1,
    location_x=None,
    location_y=None,
    m=2.0,
    max_iter_fcm=30,
    max_iter_gk=30,
):
    """Train FMP model with MATLAB-equivalent data representation and clustering."""
    traj = np.asarray(demo_traj, dtype=float)
    t = np.asarray(time_axis, dtype=float).reshape(-1)
    if traj.ndim != 2:
        raise ValueError("demo_traj must be a 2D array shaped (D_out, N).")
    if t.size != traj.shape[1]:
        raise ValueError("time_axis length must equal demo_traj sample count.")
    if t.size < 2:
        raise ValueError("time_axis requires at least 2 samples.")

    d_out = traj.shape[0]
    if location_x is None:
        location_x_arr = np.array([0, 1], dtype=int)
    else:
        location_x_arr = np.asarray(location_x, dtype=int).reshape(-1)
    if location_y is None:
        location_y_arr = np.arange(2, 2 + d_out, dtype=int)
    else:
        location_y_arr = np.asarray(location_y, dtype=int).reshape(-1)

    dt = float(np.median(np.diff(t)))
    demo_dura = 1.0 / max(dt, 1e-9)
    data_train = _build_fmp_data(traj, t, alpha=float(alpha), demo_dura=demo_dura)

    c, inv_covs, u = gk_clustering(
        data=data_train,
        n_clusters=int(N_C),
        m=m,
        max_iter_fcm=int(max_iter_fcm),
        max_iter_gk=int(max_iter_gk),
    )
    p1_u = _train_local_regression(
        data_train=data_train,
        u=u,
        n_clusters=int(N_C),
        location_x=location_x_arr,
        location_y=location_y_arr,
    )

    return {
        "C": c,
        "inv_covs": inv_covs,
        "p1_u": p1_u,
        "alpha": float(alpha),
        "N_C": int(N_C),
        "location_x": location_x_arr,
        "location_y": location_y_arr,
        "m": float(m),
        "demo_dura": float(demo_dura),
    }


def modulate_trajectory(
    fmp_model,
    demo_traj,
    time_axis,
    via_points,
    via_times,
    transition_ratio=0.1,
    transition_gamma=1.0,
):
    """
    MATLAB-equivalent modulation with Switch + Add + Soft Boundary Blending.
    Returns trajectory with shape (D_out, N).
    """
    traj = np.asarray(demo_traj, dtype=float)
    t = np.asarray(time_axis, dtype=float).reshape(-1)
    if traj.ndim != 2 or traj.shape[1] != t.size:
        raise ValueError("demo_traj must be (D_out, N) and match time_axis length.")

    c1 = np.asarray(fmp_model["C"], dtype=float)
    p_inv_cov1 = np.asarray(fmp_model["inv_covs"], dtype=float)
    p1_u1 = np.asarray(fmp_model["p1_u"], dtype=float)
    alpha = float(fmp_model["alpha"])
    n_c = int(fmp_model["N_C"])
    location_x = np.asarray(fmp_model.get("location_x", [0, 1]), dtype=int).reshape(-1)
    location_y = np.asarray(fmp_model.get("location_y", np.arange(2, 2 + traj.shape[0])), dtype=int).reshape(-1)
    m = float(fmp_model.get("m", 2.0))
    demo_dura = float(fmp_model.get("demo_dura", 1.0 / max(float(np.median(np.diff(t))), 1e-9)))

    data_test = _build_fmp_data(traj, t, alpha=alpha, demo_dura=demo_dura)
    n_data = data_test.shape[1]
    d_out = location_y.size

    via_p = np.asarray(via_points, dtype=float)
    via_t = np.asarray(via_times, dtype=float).reshape(-1)
    if via_p.ndim != 2:
        via_p = via_p.reshape(traj.shape[0], -1)
    n_via = via_t.size if via_p.size > 0 else 0

    transition_ratio = float(max(0.0, min(0.5, transition_ratio)))
    transition_gamma = float(max(1.0, transition_gamma))

    if n_via <= 0:
        md2 = _compute_md2(
            data_sel=data_test[location_x, :],
            centers_sel=c1[location_x, :],
            inv_covs_sel=p_inv_cov1[np.ix_(location_x, location_x, np.arange(n_c))],
        )
        temp = 1.0 / np.power(md2, 1.0 / max(m - 1.0, 1e-9))
        u = temp / np.sum(temp, axis=0, keepdims=True)
        return _predict_from_membership(data_test, p1_u1, u, location_x, d_out)

    # Step 1: U_via over full feature space [time, phase, outputs].
    via_time_scaled = _align_via_times_to_data_axis(
        via_times=via_t,
        raw_time_axis=t,
        data_time_axis=data_test[0, :],
        demo_dura=demo_dura,
    )
    via_phase = np.exp(alpha * via_time_scaled / max(float(data_test[0, -1]), 1e-9))
    via_stack = np.vstack((via_time_scaled.reshape(1, -1), via_phase.reshape(1, -1), via_p))

    md2_via = np.zeros((n_c, n_via), dtype=float)
    for i in range(n_c):
        dist = via_stack - c1[:, i : i + 1]
        md2_via[i, :] = np.sum(dist * (p_inv_cov1[:, :, i] @ dist), axis=0)
    md2_via = np.maximum(md2_via, 1e-12)
    temp = 1.0 / md2_via
    u_via = temp / np.sum(temp, axis=0, keepdims=True)
    jl = np.argmax(u_via, axis=0)

    # Step 2: classify switch vs add.
    via_time1 = [via_time_scaled[0]]
    via_point1 = [via_p[:, 0]]
    jl1 = [int(jl[0])]
    via_time2 = []
    via_point2 = []
    u_via2 = []
    jl2 = []
    seen = {int(jl[0])}
    for i in range(1, n_via):
        key = int(jl[i])
        if key in seen:
            via_time2.append(via_time_scaled[i])
            via_point2.append(via_p[:, i])
            u_via2.append(u_via[:, i].copy())
            jl2.append(key)
        else:
            seen.add(key)
            via_time1.append(via_time_scaled[i])
            via_point1.append(via_p[:, i])
            jl1.append(key)

    via_point1_arr = np.column_stack(via_point1)
    via_time1_arr = np.asarray(via_time1, dtype=float)

    # Step 3: switch.
    c = c1.copy()
    p1_u = p1_u1.copy()
    for i in range(via_time1_arr.size):
        target_t = via_time1_arr[i]
        idx = int(np.argmin(np.abs(data_test[0, :] - target_t)))
        c_index = int(jl1[i])

        c[:-1, c_index] = data_test[:-1, idx] + 1e-7
        datax = data_test[location_x, idx]
        slope = p1_u[1:, :, c_index]
        p1_u[0, :, c_index] = via_point1_arr[:, i] - datax @ slope

    # Step 4: add.
    c2 = c.copy()
    p_inv_cov2 = p_inv_cov1.copy()
    p1_u2 = p1_u.copy()
    n_c1 = n_c

    if via_time2:
        via_time2_arr = np.asarray(via_time2, dtype=float)
        via_point2_arr = np.column_stack(via_point2)
        u_via2_arr = np.column_stack(u_via2)

        add_count = via_time2_arr.size
        c2 = np.hstack([c2, np.zeros((c2.shape[0], add_count), dtype=float)])
        p_inv_cov2 = np.concatenate(
            [p_inv_cov2, np.zeros((p_inv_cov2.shape[0], p_inv_cov2.shape[1], add_count), dtype=float)],
            axis=2,
        )
        p1_u2 = np.concatenate(
            [p1_u2, np.zeros((p1_u2.shape[0], p1_u2.shape[1], add_count), dtype=float)],
            axis=2,
        )

        for i in range(add_count):
            target_t = via_time2_arr[i]
            idx = int(np.argmin(np.abs(data_test[0, :] - target_t)))
            new_idx = n_c1 + i

            c2[:, new_idx] = c1[:, jl2[i]]
            c2[:-1, new_idx] = data_test[:-1, idx] + 1e-7

            weights = u_via2_arr[:, i].reshape(1, 1, -1)
            p_inv_cov2[:, :, new_idx] = np.sum(p_inv_cov1 * weights, axis=2)

            p1_u2[:, :, new_idx] = p1_u1[:, :, jl2[i]]
            slope = p1_u2[1:, :, new_idx]
            datax = data_test[location_x, idx]
            p1_u2[0, :, new_idx] = via_point2_arr[:, i] - datax @ slope

        n_c1 += add_count

    # Step 5: full MD2 with location_x.
    md2 = _compute_md2(
        data_sel=data_test[location_x, :],
        centers_sel=c2[location_x, :n_c1],
        inv_covs_sel=p_inv_cov2[np.ix_(location_x, location_x, np.arange(n_c1))],
    )
    temp = 1.0 / np.power(md2, 1.0 / max(m - 1.0, 1e-9))

    # Step 6: soft boundary blending on added clusters.
    if n_c1 > n_c:
        t_data = data_test[0, :]
        core_start = float(np.min(via_time_scaled))
        core_end = float(np.max(via_time_scaled))
        total_time = max(float(t_data[-1] - t_data[0]), 1e-12)
        transition_len = transition_ratio * total_time
        eps_mask = 1e-6

        decay_mask = np.full(n_data, eps_mask, dtype=float)
        core_idx = (t_data >= core_start) & (t_data <= core_end)
        decay_mask[core_idx] = 1.0

        if transition_len > 0.0:
            left_start = core_start - transition_len
            left_idx = (t_data >= left_start) & (t_data < core_start)
            if np.any(left_idx):
                s = (t_data[left_idx] - left_start) / transition_len
                blend = 0.5 - 0.5 * np.cos(np.pi * s)
                decay_mask[left_idx] = eps_mask + (1.0 - eps_mask) * (blend**transition_gamma)

            right_end = core_end + transition_len
            right_idx = (t_data > core_end) & (t_data <= right_end)
            if np.any(right_idx):
                s = (t_data[right_idx] - core_end) / transition_len
                blend = 0.5 + 0.5 * np.cos(np.pi * s)
                decay_mask[right_idx] = eps_mask + (1.0 - eps_mask) * (blend**transition_gamma)

        temp[n_c:n_c1, :] *= decay_mask[None, :]

    # Step 7: normalize and predict.
    u = temp / np.sum(temp, axis=0, keepdims=True)
    return _predict_from_membership(data_test, p1_u2[:, :, :n_c1], u, location_x, d_out)
