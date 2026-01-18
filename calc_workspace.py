
import sys
import os
import numpy as np
from scipy.spatial.transform import Rotation as R

# Add current directory to path so we can import robot.kinematics
current_dir = os.getcwd()
if current_dir not in sys.path:
    sys.path.append(current_dir)

try:
    from robot.kinematics import RobotKinematics
except ImportError:
    # Fallback if running from a different CWD
    sys.path.append(r"c:\Users\jakub\Desktop\PAROL6_GUI")
    from robot.kinematics import RobotKinematics

def analyze_workspace():
    urdf_path = r"c:\Users\jakub\Desktop\PAROL6_GUI\assets\PAROL6.urdf"
    
    print(f"Initializing Kinematics with URDF: {urdf_path}")
    ik = RobotKinematics(urdf_path)
    
    # Get limits in radians
    # Define limits explicitly as requested
    joint_limits = {
        "J1": (-90, 90), "J2": (-50, 140), "J3": (-100, 70),
        "J4": (-100, 180), "J5": (-120, 110), "J6": (-110, 180) 
    }
    
    # Convert to radians for calculation
    limits_rad = []
    for i in range(1, 7):
        mn, mx = joint_limits[f"J{i}"]
        limits_rad.append((np.deg2rad(mn), np.deg2rad(mx)))
    
    tools = ["CHWYTAK_MALY", "CHWYTAK_DUZY"]
    tool_names = {
        "CHWYTAK_MALY": "PNEUMATIC GRIPPER (Vacuum)",
        "CHWYTAK_DUZY": "ELECTRIC GRIPPER (Stepper)"
    }

    N_SAMPLES = 100000
    print(f"Sampling {N_SAMPLES} random poses per tool...")

    for tool_key in tools:
        print(f"\nAnalyzing Tool: {tool_names[tool_key]}...")
        ik.set_tool(tool_key)
        
        # Arrays to store results
        X_vals, Y_vals, Z_vals = [], [], []
        A_vals, B_vals, C_vals = [], [], [] # XYZ Euler
        
        # Vectorized generation
        # 0. Create random samples (N x 6)
        samples = np.zeros((N_SAMPLES, 6))
        for j in range(6):
            low, high = limits_rad[j]
            samples[:, j] = np.random.uniform(low, high, N_SAMPLES)
        
        # 1. Calculate FK for each
        # Since FK is not vectorized in RobotKinematics (it uses chain.forward_kinematics which might support it, but
        # RobotKinematics wraps it with tooling offsets), we do a loop or optimize.
        # RobotKinematics.forward_kinematics takes 1D array.
        
        # We'll simple loop. 100k is fast enough for compiled numpy but slow for python loop.
        # Let's reduce comparison overhead by batching or just accept 5-10s runtime.
        
        for i in range(N_SAMPLES):
            joints = samples[i]
            
            # --- FK Logic Copy to avoid overhead of method call if possible, or just call it ---
            # Calling the method is safer to ensure logic match.
            tcp_matrix = ik.forward_kinematics(joints)
            
            # Position
            pos = tcp_matrix[:3, 3] # meters
            X_vals.append(pos[0])
            Y_vals.append(pos[1])
            Z_vals.append(pos[2])
            
            # Orientation
            rot_mat = tcp_matrix[:3, :3]
            # Convert to Euler XYZ degrees
            # Note: Euler angles are cyclic, so min/max might be misleading if they wrap around.
            # But user asked for range.
            euler = R.from_matrix(rot_mat).as_euler('xyz', degrees=True)
            A_vals.append(euler[0])
            B_vals.append(euler[1])
            C_vals.append(euler[2])
            
            if i % 10000 == 0:
                print(f"  Processed {i}/{N_SAMPLES}...", end='\r')

        # Convert to arrays
        X_vals = np.array(X_vals) * 1000.0 # to mm
        Y_vals = np.array(Y_vals) * 1000.0
        Z_vals = np.array(Z_vals) * 1000.0
        
        A_vals = np.array(A_vals)
        B_vals = np.array(B_vals)
        C_vals = np.array(C_vals)

        # --- FILTERING VALID WORKSPACE (Z > 0) ---
        # User defined limits are very wide, causing robot to go below Z=0.
        # We filter this to show "Tabletop" workspace.
        valid_mask = Z_vals >= 0.0
        n_valid = np.sum(valid_mask)
        print(f"  Valid Poses (Z >= 0): {n_valid}/{N_SAMPLES} ({(n_valid/N_SAMPLES)*100:.1f}%)") # Show how many were underground

        if n_valid == 0:
            print("  [WARNING] NO VALID POSES FOUND ABOVE Z=0.")
            continue

        X_f = X_vals[valid_mask]
        Y_f = Y_vals[valid_mask]
        Z_f = Z_vals[valid_mask]
        A_f = A_vals[valid_mask]
        B_f = B_vals[valid_mask]
        C_f = C_vals[valid_mask]

        # Calculate XY Radius (Reach)
        radii = np.sqrt(X_f**2 + Y_f**2)

        print("-" * 60)
        print(f"WORKSPACE LIMITS FOR: {tool_names[tool_key]} (Filtered Z>=0)")
        print("-" * 60)
        print(f"X:         Min: {X_f.min():8.2f} mm | Max: {X_f.max():8.2f} mm | Span: {(X_f.max()-X_f.min()):8.2f} mm")
        print(f"Y:         Min: {Y_f.min():8.2f} mm | Max: {Y_f.max():8.2f} mm | Span: {(Y_f.max()-Y_f.min()):8.2f} mm")
        print(f"Z:         Min: {Z_f.min():8.2f} mm | Max: {Z_f.max():8.2f} mm | Span: {(Z_f.max()-Z_f.min()):8.2f} mm")
        print(f"XY Reach:  Max Radius: {radii.max():8.2f} mm (from base center)")
        print("-" * 20)
        print(f"A (Roll):  Min: {A_f.min():8.2f}° | Max: {A_f.max():8.2f}°")
        print(f"B (Pitch): Min: {B_f.min():8.2f}° | Max: {B_f.max():8.2f}°")
        print(f"C (Yaw):   Min: {C_f.min():8.2f}° | Max: {C_f.max():8.2f}°")
        print("-" * 60)

if __name__ == "__main__":
    analyze_workspace()
