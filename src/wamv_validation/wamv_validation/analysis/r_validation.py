#!/usr/bin/env python3
import argparse
import numpy as np
import pandas as pd
from scipy.integrate import solve_ivp
from scipy.optimize import minimize
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt

# ---------------------------
# Default vessel params (VRX / WAM-V baseline)
# ---------------------------
m = 180.0
Iz = 446.0

# initial added-mass guesses
X_udot_init = -125.0
Y_vdot_init = -125.0
N_rdot = -250.0

# Linear damping
X_u = 100.0
Y_v = 100.0
N_r = 800.0

# Quadratic damping
X_uu = 150.0
Y_vv = 100.0
N_rr = 800.0

# Geometry
beam = 2.0
y_left = +beam / 2.0
y_right = -beam / 2.0  # right thruster -> negative yaw torque

# ------------------------------------------------
# Thruster input (right thruster on for on_time seconds)
# ------------------------------------------------
def thrust_pattern_right(t, thrust=100.0, on_time=10.0):
    return thrust if t < on_time else 0.0

# ------------------------------------------------
# Fossen 3-DOF ODE (uses X_udot and Y_vdot passed in via closure)
# state: [u, v, r, psi, x, y]
# ------------------------------------------------
def make_odefun(X_udot, Y_vdot, N_rdot=0.0, thrust=100.0, on_time=10.0):
    def odefun(t, state):
        u, v, r, psi, x, y = state
        T_right = thrust_pattern_right(t, thrust=thrust, on_time=on_time)
        T_x = T_right
        T_y = 0.0
        tau_z = y_right * T_right

        M = np.array([
            [m - X_udot, 0.0, 0.0],
            [0.0, m - Y_vdot, 0.0],
            [0.0, 0.0, Iz - N_rdot]
        ])

        C = np.array([
            [0.0, 0.0, -(m - Y_vdot) * v],
            [0.0, 0.0,  (m - X_udot) * u],
            [(m - Y_vdot) * v, -(m - X_udot) * u, 0.0]
        ])

        D = np.diag([
            X_u + X_uu * abs(u),
            Y_v + Y_vv * abs(v),
            N_r + N_rr * abs(r)
        ])

        tau = np.array([T_x, T_y, tau_z])
        nu = np.array([u, v, r])

        nu_dot = np.linalg.solve(M, tau - C @ nu - D @ nu)
        u_dot, v_dot, r_dot = nu_dot

        x_dot = u * np.cos(psi) - v * np.sin(psi)
        y_dot = u * np.sin(psi) + v * np.cos(psi)
        psi_dot = r

        return [u_dot, v_dot, r_dot, psi_dot, x_dot, y_dot]
    return odefun

# ------------------------------------------------
# Evaluate Fossen accelerations for given states (helper)
# ------------------------------------------------
def compute_accel_from_states(u, v, r, X_udot, Y_vdot, N_rdot=0.0, thrust_x=0.0, tau_z=0.0):
    M = np.array([
        [m - X_udot, 0.0, 0.0],
        [0.0, m - Y_vdot, 0.0],
        [0.0, 0.0, Iz - N_rdot]
    ])
    C = np.array([
        [0.0, 0.0, -(m - Y_vdot) * v],
        [0.0, 0.0,  (m - X_udot) * u],
        [(m - Y_vdot) * v, -(m - X_udot) * u, 0.0]
    ])
    D = np.diag([
        X_u + X_uu * abs(u),
        Y_v + Y_vv * abs(v),
        N_r + N_rr * abs(r)
    ])
    tau = np.array([thrust_x, 0.0, tau_z])
    nu = np.array([u, v, r])
    return np.linalg.solve(M, tau - C @ nu - D @ nu)  # returns [u_dot, v_dot, r_dot]

# ------------------------------------------------
# Read odometry CSV (ROS format) and compute measured accelerations
#   - adaptive Savitzky-Golay derivative is applied ONLY to measured accelerations
# ------------------------------------------------
def read_odom_ros_csv(path, sg_poly=3, sg_pct=0.05):
    df = pd.read_csv(path)

    # drop exact duplicate stamps if any (prevents zero dt)
    df = df.drop_duplicates(subset=["_header._stamp._sec", "_header._stamp._nanosec"], keep='first').reset_index(drop=True)

    # time in seconds (float)
    t = df["_header._stamp._sec"].astype(float).to_numpy() + df["_header._stamp._nanosec"].astype(float).to_numpy() * 1e-9
    t = np.maximum.accumulate(t)  # ensure monotonic
    t = t - t[0]

    # velocities
    u = df["_twist._twist._linear._x"].astype(float).to_numpy()
    v = df["_twist._twist._linear._y"].astype(float).to_numpy()
    r = df["_twist._twist._angular._z"].astype(float).to_numpy()

    # safeguard dt
    if len(t) > 1:
        dt = np.diff(t)
        dt_mean = np.mean(dt[dt > 0]) if np.any(dt > 0) else 1e-3
    else:
        dt_mean = 1e-3

    # integrate pose (simple rectangle rule)
    dt_full = np.diff(np.insert(t, 0, 0.0))
    psi = np.cumsum(r * dt_full)
    x = np.cumsum((u * np.cos(psi) - v * np.sin(psi)) * dt_full)
    y = np.cumsum((u * np.sin(psi) + v * np.cos(psi)) * dt_full)

    # adaptive SG window: percentage of points (sg_pct), must be odd and >=5
    n = len(u)
    sg_window = max(5, int(np.round(n * sg_pct)))
    if sg_window % 2 == 0:
        sg_window += 1
    if sg_window >= n:
        sg_window = n - 1 if (n - 1) % 2 == 1 else n - 2
        if sg_window < 3:
            sg_window = 3

    # compute measured accelerations using SG derivative if enough points
    if n >= sg_window and sg_window >= 3:
        # savgol_filter with deriv=1 returns derivative; delta must be sampling spacing
        u_dot = savgol_filter(u, sg_window, sg_poly, deriv=1, delta=dt_mean)
        v_dot = savgol_filter(v, sg_window, sg_poly, deriv=1, delta=dt_mean)
        r_dot = savgol_filter(r, sg_window, sg_poly, deriv=1, delta=dt_mean)
    else:
        # fallback to gradient with respect to t (handles small data)
        # np.gradient requires strictly increasing t; we've guarded against duplicates above
        u_dot = np.gradient(u, t, edge_order=2) if len(t) > 1 else np.zeros_like(u)
        v_dot = np.gradient(v, t, edge_order=2) if len(t) > 1 else np.zeros_like(v)
        r_dot = np.gradient(r, t, edge_order=2) if len(t) > 1 else np.zeros_like(r)

    return {
        "time": t,
        "u": u, "v": v, "r": r,
        "psi": psi, "x": x, "y": y,
        "u_dot": u_dot, "v_dot": v_dot, "r_dot": r_dot
    }

# ------------------------------------------------
# R^2 metric
# ------------------------------------------------
def r2(y_true, y_pred):
    y_true = np.asarray(y_true)
    y_pred = np.asarray(y_pred)
    if y_true.size == 0:
        return np.nan
    ss_res = np.sum((y_true - y_pred)**2)
    ss_tot = np.sum((y_true - np.mean(y_true))**2)
    return 1.0 - ss_res / ss_tot if ss_tot > 0 else np.nan

# ------------------------------------------------
# Objective function for fitting
# ------------------------------------------------
def fit_objective(params, odom, t_eval, thrust=100.0, on_time=10.0):
    X_udot_try, Y_vdot_try = params
    odefun = make_odefun(X_udot_try, Y_vdot_try, N_rdot, thrust=thrust, on_time=on_time)
    state0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    sol = solve_ivp(odefun, (t_eval[0], t_eval[-1]), state0, t_eval=t_eval, method='LSODA')

    # ENU flip for predicted sway
    u_pred = sol.y[0]
    v_pred = -sol.y[1]

    u_m_interp = np.interp(sol.t, odom["time"], odom["u"])
    v_m_interp = np.interp(sol.t, odom["time"], odom["v"])

    R2_u = r2(u_m_interp, u_pred)
    R2_v = r2(v_m_interp, v_pred)

    if np.isnan(R2_u) or np.isnan(R2_v):
        return 1e6
    return -(R2_u + R2_v)

# ------------------------------------------------
# Main
# ------------------------------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--odom', required=True, help="path to odometry CSV (ROS exported)")
    parser.add_argument('--thrust', type=float, default=100.0)
    parser.add_argument('--on_time', type=float, default=9.5)
    parser.add_argument('--t_final', type=float, default=20.0)
    parser.add_argument('--n_samples', type=int, default=600)
    args = parser.parse_args()

    odom = read_odom_ros_csv(args.odom)
    t_eval = np.linspace(0.0, args.t_final, args.n_samples)

    # initial guess and bounds
    x0 = np.array([X_udot_init, Y_vdot_init])
    bounds = [(-500.0, -1.0), (-500.0, -1.0)]

    print("Fitting X_udot and Y_vdot...")
    res = minimize(fit_objective, x0, args=(odom, t_eval, args.thrust, args.on_time),
                   bounds=bounds, method='L-BFGS-B', options={'maxiter': 200})
    X_udot_fit, Y_vdot_fit = res.x
    print(f"Optimization success: {res.success}, message: {res.message}")
    print(f"Fitted X_udot = {X_udot_fit:.3f}, Y_vdot = {Y_vdot_fit:.3f}")

    # simulate fitted model
    odefun_fitted = make_odefun(X_udot_fit, Y_vdot_fit, N_rdot, thrust=args.thrust, on_time=args.on_time)
    state0 = [0.0]*6
    sol = solve_ivp(odefun_fitted, (t_eval[0], t_eval[-1]), state0, t_eval=t_eval, method='LSODA')

    # model accelerations
    u_dot_model = np.zeros_like(sol.t)
    v_dot_model = np.zeros_like(sol.t)
    r_dot_model = np.zeros_like(sol.t)
    for i, (uu, vv, rr, tt) in enumerate(zip(sol.y[0], sol.y[1], sol.y[2], sol.t)):
        T_right = thrust_pattern_right(tt, thrust=args.thrust, on_time=args.on_time)
        tau_z = y_right * T_right
        acc = compute_accel_from_states(uu, vv, rr, X_udot_fit, Y_vdot_fit, N_rdot,
                                        thrust_x=T_right, tau_z=tau_z)
        u_dot_model[i], v_dot_model[i], r_dot_model[i] = acc

    # Apply ENU flips for plotting/comparison
    u_pred = sol.y[0]
    v_pred = -sol.y[1]
    r_pred = -sol.y[2]
    psi_pred = -sol.y[3]
    x_pred = sol.y[4]
    y_pred = -sol.y[5]
    u_dot_pred = u_dot_model
    v_dot_pred = -v_dot_model
    r_dot_pred = -r_dot_model

    # interpolate measured signals to model time grid
    u_m = np.interp(sol.t, odom["time"], odom["u"])
    v_m = np.interp(sol.t, odom["time"], odom["v"])
    r_m = np.interp(sol.t, odom["time"], odom["r"])
    x_m = np.interp(sol.t, odom["time"], odom["x"])
    y_m = np.interp(sol.t, odom["time"], odom["y"])
    u_dot_m = np.interp(sol.t, odom["time"], odom["u_dot"])
    v_dot_m = np.interp(sol.t, odom["time"], odom["v_dot"])
    r_dot_m = np.interp(sol.t, odom["time"], odom["r_dot"])
    psi_m = np.interp(sol.t, odom["time"], odom["psi"])

    # compute R^2 metrics
    R2_u = r2(u_m, u_pred)
    R2_v = r2(v_m, v_pred)
    R2_r = r2(r_m, r_pred)
    R2_x = r2(x_m, x_pred)
    R2_y = r2(y_m, y_pred)
    R2_du = r2(u_dot_m, u_dot_pred)
    R2_dv = r2(v_dot_m, v_dot_pred)
    R2_dr = r2(r_dot_m, r_dot_pred)
    R2_psi = r2(psi_m, psi_pred)

    # save CSV
    out_df = pd.DataFrame({
        "time": sol.t,
        "u_pred": u_pred,
        "v_pred": v_pred,
        "r_pred": r_pred,
        "psi_pred": psi_pred,
        "x_pred": x_pred,
        "y_pred": y_pred,
        "u_dot_model": u_dot_pred,
        "v_dot_model": v_dot_pred,
        "r_dot_model": r_dot_pred
    })
    out_csv = "wamv_pred_fitted.csv"
    out_df.to_csv(out_csv, index=False)
    print(f"✔ wrote predictions to {out_csv}")

    # ---------------------------
    # Plotting (R² in legend)
    # ---------------------------
    plt.figure(figsize=(12,12))

    plt.subplot(3,3,1)
    plt.plot(sol.t, u_pred, 'r--', label=f'Pred u (R²={R2_u:.3f})')
    plt.plot(sol.t, u_m, 'k', label='Meas u')
    plt.ylabel("u (m/s)"); plt.legend()

    plt.subplot(3,3,2)
    plt.plot(sol.t, -v_pred, 'b--', label=f'Pred v (R²={R2_v:.3f})')
    plt.plot(sol.t, -v_m, 'k', label='Meas v')
    plt.ylabel("v (m/s)"); plt.legend()

    plt.subplot(3,3,3)
    plt.plot(sol.t, r_pred, 'm--', label=f'Pred r (R²={R2_r:.3f})')
    plt.plot(sol.t, r_m, 'k', label='Meas r')
    plt.ylabel("r (rad/s)"); plt.legend()

    plt.subplot(3,3,4)
    plt.plot(sol.t, u_dot_pred, 'r--', label=f'Pred a_u (R²={R2_du:.3f})')
    plt.plot(sol.t, u_dot_m, 'k', label='Meas a_u')
    plt.ylabel("a_u (m/s²)"); plt.legend()

    plt.subplot(3,3,5)
    plt.plot(sol.t, -v_dot_pred, 'b--', label=f'Pred a_v (R²={R2_dv:.3f})')
    plt.plot(sol.t, -v_dot_m, 'k', label='Meas a_v')
    plt.ylabel("a_v (m/s²)"); plt.legend()

    plt.subplot(3,3,6)
    plt.plot(sol.t, r_dot_pred, 'm--', label=f'Pred r_dot (R²={R2_dr:.3f})')
    plt.plot(sol.t, r_dot_m, 'k', label='Meas r_dot')
    plt.ylabel("r_dot (rad/s²)"); plt.legend()

    plt.subplot(3,3,7)
    plt.plot(sol.t, x_pred, 'r--', label=f'Pred x (R²={R2_x:.3f})')
    plt.plot(sol.t, x_m, 'k', label='Meas x')
    plt.ylabel("x (m)"); plt.xlabel("t"); plt.legend()

    plt.subplot(3,3,8)
    plt.plot(sol.t, y_pred, 'b--', label=f'Pred y (R²={R2_y:.3f})')
    plt.plot(sol.t, y_m, 'k', label='Meas y')
    plt.ylabel("y (m)"); plt.xlabel("t"); plt.legend()

    plt.subplot(3,3,9)
    plt.plot(sol.t, psi_pred, 'g--', label=f'Pred psi (R²={R2_psi:.3f})')
    plt.plot(sol.t, psi_m, 'k', label='Meas psi')
    plt.ylabel("psi (rad)"); plt.xlabel("t"); plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
