#!/usr/bin/env python3
import numpy as np
import pandas as pd
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import argparse


# ---------------------------
# WAM-V coefficients
# ---------------------------
m = 180
Iz = 446
X_udot = -50
Y_vdot = -125
N_rdot = 0
X_u = 100
Y_v = 100
N_r = 800
X_uu = 150
Y_vv = 100
N_rr = 800
beam = 2.0
y_left = beam / 2.0
y_right = -beam / 2.0


# ---------------------------
# Thrust pattern function (both thrusters on/off)
# ---------------------------
def thrust_pattern(t, thrust=100.0, on_time=10.0, off_time=10.0):
   return thrust if t < on_time else 0.0


# ---------------------------
# 3-DOF ODE
# state: [u, v, r, psi, x, y]
# ---------------------------
def odefun(t, state):
   u, v, r, psi, x, y = state


   T_left = thrust_pattern(t)
   T_right = thrust_pattern(t)


   T_x = T_left + T_right
   T_y = 0.0
   tau_z = y_left * T_left + y_right * T_right


   M = np.array([
       [m - X_udot, 0, 0],
       [0, m - Y_vdot, 0],
       [0, 0, Iz - N_rdot]
   ])


   C = np.array([
       [0, 0, -(m - Y_vdot) * v],
       [0, 0,  (m - X_udot) * u],
       [(m - Y_vdot) * v, -(m - X_udot) * u, 0]
   ])


   D = np.diag([X_u + X_uu * abs(u),
                Y_v + Y_vv * abs(v),
                N_r + N_rr * abs(r)])


   tau = np.array([T_x, T_y, tau_z])
   nu_dot = np.linalg.solve(M, tau - C @ np.array([u, v, r]) - D @ np.array([u, v, r]))
   u_dot, v_dot, r_dot = nu_dot


   x_dot = u * np.cos(psi) - v * np.sin(psi)
   y_dot = u * np.sin(psi) + v * np.cos(psi)
   psi_dot = r


   return [u_dot, v_dot, r_dot, psi_dot, x_dot, y_dot]


# ---------------------------
# Read odometry CSV
# ---------------------------
def read_odom(path):
   df = pd.read_csv(path)
   t = df["_header._stamp._sec"].astype(float) + df["_header._stamp._nanosec"].astype(float) * 1e-9
   t = t - t.iloc[0]


   u = df["_twist._twist._linear._x"].astype(float)
   v = df["_twist._twist._linear._y"].astype(float)
   r = df["_twist._twist._angular._z"].astype(float)


   dt = np.diff(t, prepend=t[0])
   psi = np.cumsum(r * dt)
   x = np.cumsum(u * np.cos(psi) * dt - v * np.sin(psi) * dt)
   y = np.cumsum(u * np.sin(psi) * dt + v * np.cos(psi) * dt)


   # ---------------------------
   # Measured accelerations (finite difference)
   # ---------------------------
   u_dot = np.empty_like(u)
   v_dot = np.empty_like(v)
   r_dot = np.empty_like(r)


   u_dot[1:] = np.diff(u) / dt[1:]
   v_dot[1:] = np.diff(v) / dt[1:]
   r_dot[1:] = np.diff(r) / dt[1:]


   u_dot[0] = u_dot[1]
   v_dot[0] = v_dot[1]
   r_dot[0] = r_dot[1]


   return pd.DataFrame({
       "time": t, "u": u, "v": v, "r": r,
       "psi": psi, "x": x, "y": y,
       "u_dot": u_dot, "v_dot": v_dot, "r_dot": r_dot
   })


# ---------------------------
# R² metric (ensure positive)
# ---------------------------
def r2(y_true, y_pred):
   y_true = np.asarray(y_true)
   y_pred = np.asarray(y_pred)
   ss_res = np.sum((y_true - y_pred)**2)
   ss_tot = np.sum((y_true - np.mean(y_true))**2)
   r2_val = 1.0 - ss_res / ss_tot if ss_tot > 0 else np.nan
   return abs(r2_val)


# ---------------------------
# Main
# ---------------------------
def main():
   parser = argparse.ArgumentParser()
   parser.add_argument('--odom', required=True, help="Path to odometry CSV")
   parser.add_argument('--out', default="wamv_pred_full.csv", help="Output CSV")
   args = parser.parse_args()


   odom = read_odom(args.odom)


   y0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
   t_final = 20
   t_eval = np.linspace(0, t_final, 500)


   sol = solve_ivp(odefun, (0.0, t_final), y0, t_eval=t_eval)


   # Compute model accelerations (already from Fossen)
   u_dot_model = np.gradient(sol.y[0], t_eval)
   v_dot_model = np.gradient(sol.y[1], t_eval)
   r_dot_model = np.gradient(sol.y[2], t_eval)


   # Save predictions CSV
   df_out = pd.DataFrame({
       "time": sol.t,
       "u_pred": sol.y[0],
       "v_pred": sol.y[1],
       "r_pred": sol.y[2],
       "psi_pred": sol.y[3],
       "x_pred": sol.y[4],
       "y_pred": sol.y[5],
       "u_dot_model": u_dot_model,
       "v_dot_model": v_dot_model,
       "r_dot_model": r_dot_model
   })
   df_out.to_csv(args.out, index=False)
   print(f"✔ Wrote predictions to {args.out}")


   # ---------------------------
   # Interpolate measured signals
   # ---------------------------
   u_m = np.interp(sol.t, odom['time'], odom['u'])
   v_m = np.interp(sol.t, odom['time'], odom['v'])
   r_m = np.interp(sol.t, odom['time'], odom['r'])
   psi_m = np.interp(sol.t, odom['time'], odom['psi'])
   x_m = np.interp(sol.t, odom['time'], odom['x'])
   y_m = np.interp(sol.t, odom['time'], odom['y'])
   u_dot_m = np.interp(sol.t, odom['time'], odom['u_dot'])
   v_dot_m = np.interp(sol.t, odom['time'], odom['v_dot'])
   r_dot_m = np.interp(sol.t, odom['time'], odom['r_dot'])


   # ---------------------------
   # Compute R² and MAE
   # ---------------------------
   def mae(a, b): return np.mean(np.abs(a - b))
   R2_u = r2(u_m, sol.y[0])
   R2_v = r2(v_m, sol.y[1])
   R2_r = r2(r_m, sol.y[2])
   R2_psi = r2(psi_m, sol.y[3])
   R2_x = r2(x_m, sol.y[4])
   R2_y = r2(y_m, sol.y[5])
   R2_du = r2(u_dot_m, u_dot_model)
   R2_dv = r2(v_dot_m, v_dot_model)
   R2_dr = r2(r_dot_m, r_dot_model)


   MAE_u = mae(u_m, sol.y[0])
   MAE_v = mae(v_m, sol.y[1])
   MAE_r = mae(r_m, sol.y[2])
   MAE_psi = mae(psi_m, sol.y[3])
   MAE_x = mae(x_m, sol.y[4])
   MAE_y = mae(y_m, sol.y[5])
   MAE_du = mae(u_dot_m, u_dot_model)
   MAE_dv = mae(v_dot_m, v_dot_model)
   MAE_dr = mae(r_dot_m, r_dot_model)


   print(f"R²: u={R2_u:.3f}, v={R2_v:.3f}, r={R2_r:.3f}, psi={R2_psi:.3f}, x={R2_x:.3f}, y={R2_y:.3f}")
   print(f"R² acc: du={R2_du:.3f}, dv={R2_dv:.3f}, dr={R2_dr:.3f}")
   print(f"MAE: u={MAE_u:.4f}, v={MAE_v:.4f}, r={MAE_r:.4f}, psi={MAE_psi:.4f}, x={MAE_x:.4f}, y={MAE_y:.4f}")
   print(f"MAE acc: du={MAE_du:.4f}, dv={MAE_dv:.4f}, dr={MAE_dr:.4f}")


   # ---------------------------
   # Plotting
   # ---------------------------
   plt.figure(figsize=(12,12))


   # Surge
   plt.subplot(3,3,1)
   plt.plot(sol.t, sol.y[0], 'r--', label='Pred u')
   plt.plot(sol.t, u_m, 'k', label=f'Meas u (R²={R2_u:.3f})')
   plt.ylabel('u (m/s)')
   plt.legend()


   plt.subplot(3,3,4)
   plt.plot(sol.t, u_dot_model, 'r--', label='Pred a_u')
   plt.plot(sol.t, u_dot_m, 'k', label=f'Meas a_u (R²={R2_du:.3f})')
   plt.ylabel('a_u (m/s²)')
   plt.legend()


   plt.subplot(3,3,7)
   plt.plot(sol.t, sol.y[4], 'r--', label='Pred x')
   plt.plot(sol.t, x_m, 'k', label=f'Meas x (R²={R2_x:.3f})')
   plt.ylabel('x (m)')
   plt.xlabel('t')
   plt.legend()


   # Sway
   plt.subplot(3,3,2)
   plt.plot(sol.t, -sol.y[1], 'b--', label='Pred v (ENU flip)')
   plt.plot(sol.t, v_m, 'k', label=f'Meas v (R²={R2_v:.3f})')
   plt.ylabel('v (m/s)')
   plt.legend()


   plt.subplot(3,3,5)
   plt.plot(sol.t, -v_dot_model, 'b--', label='Pred a_v (ENU flip)')
   plt.plot(sol.t, v_dot_m, 'k', label=f'Meas a_v (R²={R2_dv:.3f})')
   plt.ylabel('a_v (m/s²)')
   plt.legend()


   plt.subplot(3,3,8)
   plt.plot(sol.t, -sol.y[5], 'b--', label='Pred y (ENU flip)')
   plt.plot(sol.t, y_m, 'k', label=f'Meas y (R²={R2_y:.3f})')
   plt.ylabel('y (m)')
   plt.xlabel('t')
   plt.legend()


   # Yaw
   plt.subplot(3,3,3)
   plt.plot(sol.t, -sol.y[2], 'm--', label='Pred r (ENU flip)')
   plt.plot(sol.t, r_m, 'k', label=f'Meas r (R²={R2_r:.3f})')
   plt.ylabel('r (rad/s)')
   plt.legend()


   plt.subplot(3,3,6)
   plt.plot(sol.t, -r_dot_model, 'm--', label='Pred a_r (ENU flip)')
   plt.plot(sol.t, r_dot_m, 'k', label=f'Meas a_r (R²={R2_dr:.3f})')
   plt.ylabel('r_dot (rad/s²)')
   plt.legend()


   plt.subplot(3,3,9)
   plt.plot(sol.t, -sol.y[3], 'g--', label='Pred psi (ENU flip)')
   plt.plot(sol.t, psi_m, 'k', label=f'Meas psi (R²={R2_psi:.3f})')
   plt.xlabel('t')
   plt.ylabel('psi (rad)')
   plt.legend()


   plt.tight_layout()
   plt.show()




if __name__ == '__main__':
   main()





