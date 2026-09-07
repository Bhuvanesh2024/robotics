#!/usr/bin/env python3
"""
3D 4-DOF Robot Arm Kinematics (Forward & Inverse Kinematics)
Interactive 3D visualization using Matplotlib mplot3d.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def forward_kinematics_3d(theta1, theta2, theta3, theta4, l1, l2, l3, l4, base_height=1.5):
    """
    Computes 3D joint positions and end-effector position.
    - theta1: Base yaw (rotation around vertical Z-axis) in radians
    - theta2: Shoulder pitch in radians
    - theta3: Elbow pitch in radians
    - theta4: Wrist pitch in radians
    - l1..l4: Link lengths
    """
    # Joint 0 (Origin / Base floor)
    p0 = np.array([0.0, 0.0, 0.0])
    
    # Joint 1 (Turntable top / Shoulder pivot)
    p1 = np.array([0.0, 0.0, base_height])

    # Cumulative pitch angles in elevation plane
    alpha2 = theta2
    alpha3 = theta2 + theta3
    alpha4 = theta2 + theta3 + theta4

    # Radial reach in planar elevation
    r1 = l1 * np.cos(alpha2)
    z1 = base_height + l1 * np.sin(alpha2)
    p2 = np.array([r1 * np.cos(theta1), r1 * np.sin(theta1), z1])

    r2 = r1 + l2 * np.cos(alpha3)
    z2 = z1 + l2 * np.sin(alpha3)
    p3 = np.array([r2 * np.cos(theta1), r2 * np.sin(theta1), z2])

    r3 = r2 + l3 * np.cos(alpha4)
    z3 = z2 + l3 * np.sin(alpha4)
    p4 = np.array([r3 * np.cos(theta1), r3 * np.sin(theta1), z3])

    # End-Effector
    r4 = r3 + l4 * np.cos(alpha4)
    z4 = z3 + l4 * np.sin(alpha4)
    p_ee = np.array([r4 * np.cos(theta1), r4 * np.sin(theta1), z4])

    return [p0, p1, p2, p3, p4, p_ee]

def inverse_kinematics_3d(target_x, target_y, target_z, l1, l2, l3, l4, base_height=1.5, max_iter=60):
    """
    Solves 3D Inverse Kinematics using Base Azimuth + Planar CCD.
    """
    # Step 1: Base yaw angle theta1
    theta1 = np.arctan2(target_y, target_x)

    # Step 2: Projected planar target
    planar_r = np.hypot(target_x, target_y)
    planar_z = target_z - base_height

    lengths = [l1, l2, l3, l4]
    max_reach = sum(lengths)
    target_dist = np.hypot(planar_r, planar_z)

    clamped = False
    if target_dist > max_reach:
        planar_r = planar_r * (max_reach / target_dist)
        planar_z = planar_z * (max_reach / target_dist)
        clamped = True

    # Step 3: Planar CCD for pitch angles (theta2, theta3, theta4)
    pitch_angles = np.zeros(4)
    pitch_angles[0] = np.radians(30)
    pitch_angles[1] = np.radians(45)
    pitch_angles[2] = np.radians(-30)
    pitch_angles[3] = np.radians(20)

    for _ in range(max_iter):
        for i in reversed(range(4)):
            cum = np.cumsum(pitch_angles)

            if i == 0:
                jx, jz = 0.0, 0.0
            else:
                jx = np.sum(lengths[:i] * np.cos(cum[:i]))
                jz = np.sum(lengths[:i] * np.sin(cum[:i]))

            ee_x = np.sum(lengths * np.cos(cum))
            ee_z = np.sum(lengths * np.sin(cum))

            err = np.hypot(planar_r - ee_x, planar_z - ee_z)
            if err < 1e-3:
                break

            v_ee = np.array([ee_x - jx, ee_z - jz])
            v_tgt = np.array([planar_r - jx, planar_z - jz])

            norm_ee = np.linalg.norm(v_ee)
            norm_tgt = np.linalg.norm(v_tgt)

            if norm_ee > 1e-6 and norm_tgt > 1e-6:
                cos_a = np.clip(np.dot(v_ee, v_tgt) / (norm_ee * norm_tgt), -1.0, 1.0)
                sin_a = (v_ee[0] * v_tgt[1] - v_ee[1] * v_tgt[0]) / (norm_ee * norm_tgt)
                d_theta = np.arctan2(sin_a, cos_a)
                pitch_angles[i] += d_theta * 0.8

    angles_deg = [np.degrees(theta1), np.degrees(pitch_angles[0]), np.degrees(pitch_angles[1]), np.degrees(pitch_angles[2])]
    return angles_deg, clamped

def plot_3d_arm(theta1_deg, theta2_deg, theta3_deg, theta4_deg, l1, l2, l3, l4, target=None):
    """
    Renders the 3D robot arm using Matplotlib 3D.
    """
    t1 = np.radians(theta1_deg)
    t2 = np.radians(theta2_deg)
    t3 = np.radians(theta3_deg)
    t4 = np.radians(theta4_deg)

    pts = forward_kinematics_3d(t1, t2, t3, t4, l1, l2, l3, l4)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]

    fig = plt.figure(figsize=(10, 9))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("3D 4-DOF Robot Arm Kinematics", fontsize=14, pad=20)

    # Base Column
    ax.plot([xs[0], xs[1]], [ys[0], ys[1]], [zs[0], zs[1]], 'k-', linewidth=6, label="Base Pedestal")

    # Links with color scheme matching 2D
    link_colors = ['r-', 'g-', 'b-', 'm-']
    for i in range(1, len(pts) - 1):
        ax.plot([xs[i], xs[i+1]], [ys[i], ys[i+1]], [zs[i], zs[i+1]], link_colors[(i-1) % 4], linewidth=4, label=f"Link {i}")

    # Joints
    for i in range(len(pts) - 1):
        ax.scatter(xs[i], ys[i], zs[i], color='black', s=60)
        ax.text(xs[i], ys[i], zs[i], f' J{i}({xs[i]:.1f},{ys[i]:.1f},{zs[i]:.1f})', fontsize=8)

    # End-Effector
    ax.scatter(xs[-1], ys[-1], zs[-1], color='red', s=100, label="End-Effector")
    ax.text(xs[-1], ys[-1], zs[-1], f' EE({xs[-1]:.2f},{ys[-1]:.2f},{zs[-1]:.2f})', fontsize=9, color='red', weight='bold')

    # Target Marker
    if target is not None:
        tx, ty, tz = target
        ax.scatter(tx, ty, tz, color='cyan', marker='*', s=180, label="Target (IK)")
        ax.plot([xs[-1], tx], [ys[-1], ty], [zs[-1], tz], 'c--', alpha=0.6)

    # Reach boundary wireframe circle on floor
    max_reach = l1 + l2 + l3 + l4
    u = np.linspace(0, 2 * np.pi, 50)
    floor_x = max_reach * np.cos(u)
    floor_y = max_reach * np.sin(u)
    ax.plot(floor_x, floor_y, np.zeros_like(floor_x), 'c--', alpha=0.3, label="Max Reach Perimeter")

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (Height m)')
    ax.set_xlim([-max_reach, max_reach])
    ax.set_ylim([-max_reach, max_reach])
    ax.set_zlim([0, max_reach + 2])
    ax.legend(loc='upper right')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    print("=" * 60)
    print("  3D 4-DOF ROBOT ARM KINEMATICS (CLI & 3D Matplotlib)")
    print("=" * 60)
    print("Select Mode:")
    print("  1. Forward Kinematics (FK) - Input angles (Yaw, Pitch1, Pitch2, Pitch3)")
    print("  2. Inverse Kinematics (IK) - Input 3D Target (X, Y, Z)")
    mode = input("Enter choice (1 or 2, default 1): ").strip()

    l1, l2, l3, l4 = 5.0, 4.0, 3.0, 2.0

    if mode == '2':
        tx = float(input("Enter Target X (e.g. 6.0): ") or 6.0)
        ty = float(input("Enter Target Y (e.g. 4.0): ") or 4.0)
        tz = float(input("Enter Target Z (e.g. 5.0): ") or 5.0)

        angles, clamped = inverse_kinematics_3d(tx, ty, tz, l1, l2, l3, l4)
        print("\n--- Solved Joint Angles ---")
        if clamped:
            print("Notice: Target is outside arm workspace! Clamped to workspace boundary.")
        print(f"Base Yaw (theta1):      {angles[0]:.2f}°")
        print(f"Shoulder Pitch (theta2): {angles[1]:.2f}°")
        print(f"Elbow Pitch (theta3):    {angles[2]:.2f}°")
        print(f"Wrist Pitch (theta4):    {angles[3]:.2f}°")

        plot_3d_arm(angles[0], angles[1], angles[2], angles[3], l1, l2, l3, l4, target=(tx, ty, tz))
    else:
        t1 = float(input("Enter Base Yaw theta1 in degrees (e.g. 45): ") or 45.0)
        t2 = float(input("Enter Shoulder Pitch theta2 in degrees (e.g. 30): ") or 30.0)
        t3 = float(input("Enter Elbow Pitch theta3 in degrees (e.g. 45): ") or 45.0)
        t4 = float(input("Enter Wrist Pitch theta4 in degrees (e.g. -20): ") or -20.0)

        pts = forward_kinematics_3d(np.radians(t1), np.radians(t2), np.radians(t3), np.radians(t4), l1, l2, l3, l4)
        ee = pts[-1]
        print(f"\nEnd-Effector 3D Position: X={ee[0]:.2f}, Y={ee[1]:.2f}, Z={ee[2]:.2f}")

        plot_3d_arm(t1, t2, t3, t4, l1, l2, l3, l4)
