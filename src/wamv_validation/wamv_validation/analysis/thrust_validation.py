import os
import pandas as pd
import numpy as np

# --- USER PARAMETERS ---
K_T = 0.00442       # thrust coefficient
D = 0.2       # propeller diameter [m]
rho = 1000      # water density [kg/m^3]

# --- DATA PATHS ---
data_dir = '/home/riplab/ripc_ws/wamv_csv'

left_thrust_csv  = os.path.join(data_dir, 'wamv_thrusters_left_thrust.csv')
right_thrust_csv = os.path.join(data_dir, 'wamv_thrusters_right_thrust.csv')
left_angvel_txt  = os.path.join(data_dir, 'left_angvel.txt')
right_angvel_txt = os.path.join(data_dir, 'right_angvel.txt')


# --- LOAD DATA ---
df_left_thrust = pd.read_csv(left_thrust_csv)
df_right_thrust = pd.read_csv(right_thrust_csv)

# --- EXTRACT THE DATA COLUMN ---
# Your CSV has columns: _data,_check_fields,timestamp
left_thrust  = df_left_thrust['_data'].values
right_thrust = df_right_thrust['_data'].values

# --- LOAD ANGULAR VELOCITY TXT ---
def load_gz_angvel(file_path):
    values = []
    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('data:'):
                try:
                    val = float(line.split('data:')[1].strip())
                    values.append(val)
                except ValueError:
                    continue
    return np.array(values)

left_ang_vel  = load_gz_angvel(left_angvel_txt)
right_ang_vel = load_gz_angvel(right_angvel_txt)

# --- MAX VALUES ---
max_left_thrust  = np.max(left_thrust)
max_right_thrust = np.max(right_thrust)
max_left_angvel  = np.max(left_ang_vel)
max_right_angvel = np.max(right_ang_vel)

# --- EXPECTED THRUST FROM ANGULAR VELOCITY ---
expected_left_thrust  = K_T * rho * D**4 * (max_left_angvel)**2
expected_right_thrust = K_T * rho * D**4 * (max_right_angvel)**2

# --- PRINT COMPARISON ---
print("Left Thruster:")
print(f"  Max Measured Thrust: {max_left_thrust:.2f} N")
print(f"  Expected Thrust from Angular Velocity: {expected_left_thrust:.2f} N")
print(f"  Error: {max_left_thrust - expected_left_thrust:.2f} N\n")

print("Right Thruster:")
print(f"  Max Measured Thrust: {max_right_thrust:.2f} N")
print(f"  Expected Thrust from Angular Velocity: {expected_right_thrust:.2f} N")
print(f"  Error: {max_right_thrust - expected_right_thrust:.2f} N")
