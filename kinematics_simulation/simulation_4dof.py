"""
EcoSort 4-DOF Robotic Manipulator - Kinematics & Workspace Simulation
Author: Md. Al-Imran

Simulates forward and closed-form inverse kinematics (D-H parameters),
evaluates reachability, and visualizes 3D manipulator postures during 
waste pick-and-place routines.
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Denavit-Hartenberg (D-H) Link Parameters (in mm)
D1 = 65.0   # Base offset height
A2 = 80.0   # Shoulder link length
A3 = 80.0   # Elbow link length
A4 = 60.0   # Wrist to end-effector grip length

def inverse_kinematics(Px, Py, Pz, phi_deg=0.0):
    """
    Closed-form geometric inverse kinematics solving joint angles (theta1, theta2, theta3, theta4)
    from equations (4) to (10) of the EcoSort paper.
    """
    phi = math.radians(phi_deg)
    
    # Eq. (4): Base azimuth
    theta1 = math.atan2(Py, Px)
    
    # Ground plane projection
    R_ground = math.sqrt(Px**2 + Py**2)
    
    # Eq. (5) & (6): Planar wrist center coordinates
    r = R_ground - A4 * math.cos(phi)
    z_prime = Pz - D1 - A4 * math.sin(phi)
    
    # Planar two-link law of cosines (Eq. 7)
    D_sq = r**2 + z_prime**2
    cos_theta3 = (D_sq - A2**2 - A3**2) / (2.0 * A2 * A3)
    
    if abs(cos_theta3) > 1.0:
        return None  # Unreachable coordinate
    
    # Eq. (8): Elbow angle (elbow-down configuration)
    sin_theta3 = math.sqrt(max(0.0, 1.0 - cos_theta3**2))
    theta3 = math.atan2(sin_theta3, cos_theta3)
    
    # Eq. (9): Shoulder angle
    alpha = math.atan2(z_prime, r)
    beta = math.atan2(A3 * math.sin(theta3), A2 + A3 * math.cos(theta3))
    theta2 = alpha - beta
    
    # Eq. (10): Wrist orientation angle
    theta4 = phi - (theta2 + theta3)
    
    return [math.degrees(theta1), math.degrees(theta2), math.degrees(theta3), math.degrees(theta4)]

def forward_kinematics(theta1_deg, theta2_deg, theta3_deg, theta4_deg):
    """
    Forward kinematics calculating the 3D Cartesian coordinates of each joint link.
    """
    th1 = math.radians(theta1_deg)
    th2 = math.radians(theta2_deg)
    th3 = math.radians(theta3_deg)
    th4 = math.radians(theta4_deg)
    
    # Base origin J0
    p0 = np.array([0.0, 0.0, 0.0])
    
    # Shoulder joint J1 (at base height d1)
    p1 = np.array([0.0, 0.0, D1])
    
    # Elbow joint J2
    r2 = A2 * math.cos(th2)
    p2 = np.array([
        r2 * math.cos(th1),
        r2 * math.sin(th1),
        D1 + A2 * math.sin(th2)
    ])
    
    # Wrist joint J3
    r3 = r2 + A3 * math.cos(th2 + th3)
    p3 = np.array([
        r3 * math.cos(th1),
        r3 * math.sin(th1),
        D1 + A2 * math.sin(th2) + A3 * math.sin(th2 + th3)
    ])
    
    # End-effector tip J4
    phi = th2 + th3 + th4
    r4 = r3 + A4 * math.cos(phi)
    p4 = np.array([
        r4 * math.cos(th1),
        r4 * math.sin(th1),
        D1 + A2 * math.sin(th2) + A3 * math.sin(th2 + th3) + A4 * math.sin(phi)
    ])
    
    return [p0, p1, p2, p3, p4]

def simulate_and_visualize():
    """
    Simulates deposition to all 3 waste bin locations and generates a 3D visualization.
    """
    targets = {
        "Pick Station (Inspection Bed)": (140.0, 0.0, 15.0),
        "Metal Bin (60 deg, R=150mm)": (150.0 * math.cos(math.radians(60)), 150.0 * math.sin(math.radians(60)), 40.0),
        "Wet Bin (120 deg, R=150mm)": (150.0 * math.cos(math.radians(120)), 150.0 * math.sin(math.radians(120)), 40.0),
        "Dry Bin (180 deg, R=150mm)": (150.0 * math.cos(math.radians(180)), 150.0 * math.sin(math.radians(180)), 40.0),
    }

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    colors = ['#1f77b4', '#d62728', '#2ca02c', '#ff7f0e']

    print("================================================================")
    print(" EcoSort Kinematics Verification & Joint Solutions")
    print("================================================================")

    for (name, (x, y, z)), color in zip(targets.items(), colors):
        sol = inverse_kinematics(x, y, z, phi_deg=-15.0)
        if sol:
            print(f"Target: {name}")
            print(f"  Coordinates: X={x:.1f} mm, Y={y:.1f} mm, Z={z:.1f} mm")
            print(f"  Joint Soln : Base={sol[0]:.1f}°, Shoulder={sol[1]:.1f}°, Elbow={sol[2]:.1f}°, Wrist={sol[3]:.1f}°\n")
            
            # Forward kinematics for link plotting
            points = forward_kinematics(*sol)
            xs = [p[0] for p in points]
            ys = [p[1] for p in points]
            zs = [p[2] for p in points]
            
            ax.plot(xs, ys, zs, '-o', label=name, color=color, linewidth=2.5, markersize=6)
            ax.scatter([x], [y], [z], color=color, s=80, marker='*')
        else:
            print(f"Target {name} is OUT OF REACHABLE WORKSPACE!")

    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title('EcoSort 4-DOF Robotic Arm: Trajectory & Workspace Postures', fontsize=12, fontweight='bold')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, linestyle='--', alpha=0.5)

    import os
    output_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'docs', 'assets'))
    os.makedirs(output_dir, exist_ok=True)
    out_path = os.path.join(output_dir, 'kinematics_workspace_simulation.png')
    plt.savefig(out_path, dpi=300)
    print(f"Plot saved successfully to: {out_path}")

if __name__ == "__main__":
    simulate_and_visualize()
