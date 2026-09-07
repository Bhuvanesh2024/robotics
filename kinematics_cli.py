#!/usr/bin/env python3
"""
Enhanced 4-DOF Robot Arm Kinematics CLI
Supports both Forward Kinematics (FK) and Inverse Kinematics (IK) with Matplotlib visualization.
Based on original user script.
"""

import numpy as np
import matplotlib.pyplot as plt

def forward_kinematics_4dof(theta1, theta2, theta3, theta4, l1, l2, l3, l4):
    """
    Calculates the end-effector position for a 4-DOF planar robot arm.
    Angles in radians.
    """
    x = l1 * np.cos(theta1) + \
        l2 * np.cos(theta1 + theta2) + \
        l3 * np.cos(theta1 + theta2 + theta3) + \
        l4 * np.cos(theta1 + theta2 + theta3 + theta4)

    y = l1 * np.sin(theta1) + \
        l2 * np.sin(theta1 + theta2) + \
        l3 * np.sin(theta1 + theta2 + theta3) + \
        l4 * np.sin(theta1 + theta2 + theta3 + theta4)
    return x, y

def inverse_kinematics_ccd_4dof(target_x, target_y, l1, l2, l3, l4, init_thetas_deg=(0, 0, 0, 0), max_iter=60, tol=1e-3):
    """
    Computes joint angles for a given end-effector target (target_x, target_y)
    using Cyclic Coordinate Descent (CCD).
    Returns angles in degrees and the remaining Euclidean distance error.
    """
    lengths = np.array([l1, l2, l3, l4], dtype=float)
    angles = np.radians(list(init_thetas_deg), dtype=float)
    max_reach = np.sum(lengths)
    target_dist = np.hypot(target_x, target_y)

    # If target is outside workspace, clamp to outer circle
    clamped = False
    if target_dist > max_reach:
        target_x = target_x * (max_reach / target_dist)
        target_y = target_y * (max_reach / target_dist)
        clamped = True

    for _ in range(max_iter):
        for i in reversed(range(4)):
            cum_angles = np.cumsum(angles)

            # Coordinate of joint i
            if i == 0:
                jx, jy = 0.0, 0.0
            else:
                jx = np.sum(lengths[:i] * np.cos(cum_angles[:i]))
                jy = np.sum(lengths[:i] * np.sin(cum_angles[:i]))

            # Coordinate of current end-effector
            ee_x = np.sum(lengths * np.cos(cum_angles))
            ee_y = np.sum(lengths * np.sin(cum_angles))

            err = np.hypot(target_x - ee_x, target_y - ee_y)
            if err <= tol:
                return np.degrees(angles), err, clamped

            # Vectors from joint i to end-effector and target
            v_ee = np.array([ee_x - jx, ee_y - jy])
            v_tgt = np.array([target_x - jx, target_y - jy])

            mag_ee = np.linalg.norm(v_ee)
            mag_tgt = np.linalg.norm(v_tgt)

            if mag_ee > 1e-6 and mag_tgt > 1e-6:
                cos_a = np.clip(np.dot(v_ee, v_tgt) / (mag_ee * mag_tgt), -1.0, 1.0)
                sin_a = (v_ee[0] * v_tgt[1] - v_ee[1] * v_tgt[0]) / (mag_ee * mag_tgt)
                d_theta = np.arctan2(sin_a, cos_a)
                angles[i] += d_theta * 0.8  # damped rotation

    cum_angles = np.cumsum(angles)
    ee_x = np.sum(lengths * np.cos(cum_angles))
    ee_y = np.sum(lengths * np.sin(cum_angles))
    err = np.hypot(target_x - ee_x, target_y - ee_y)
    return np.degrees(angles), err, clamped

def plot_arm(theta1_deg, theta2_deg, theta3_deg, theta4_deg, l1, l2, l3, l4, target=None):
    """
    Plots the 4-DOF robot arm matching the user's visual style.
    """
    t1 = np.radians(theta1_deg)
    t2 = np.radians(theta2_deg)
    t3 = np.radians(theta3_deg)
    t4 = np.radians(theta4_deg)

    # Joint positions
    x0, y0 = 0.0, 0.0
    x1 = l1 * np.cos(t1)
    y1 = l1 * np.sin(t1)

    x2 = x1 + l2 * np.cos(t1 + t2)
    y2 = y1 + l2 * np.sin(t1 + t2)

    x3 = x2 + l3 * np.cos(t1 + t2 + t3)
    y3 = y2 + l3 * np.sin(t1 + t2 + t3)

    x4 = x3 + l4 * np.cos(t1 + t2 + t3 + t4)
    y4 = y3 + l4 * np.sin(t1 + t2 + t3 + t4)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_title("4-DOF Robot Arm Kinematics", pad=20)

    # Center axes at (0,0)
    ax.spines['left'].set_position('zero')
    ax.spines['bottom'].set_position('zero')
    ax.spines['right'].set_color('none')
    ax.spines['top'].set_color('none')

    # Plot links matching original color scheme
    ax.plot([x0, x1], [y0, y1], 'r-', linewidth=3, label="Link 1 (Red)")
    ax.plot([x1, x2], [y1, y2], 'g-', linewidth=3, label="Link 2 (Green)")
    ax.plot([x2, x3], [y2, y3], 'b-', linewidth=3, label="Link 3 (Blue)")
    ax.plot([x3, x4], [y3, y4], 'm-', linewidth=3, label="Link 4 (Magenta)")

    # Plot joints
    ax.plot(x0, y0, 'ko', markersize=8)
    ax.plot(x1, y1, 'ko', markersize=8)
    ax.plot(x2, y2, 'ko', markersize=8)
    ax.plot(x3, y3, 'ko', markersize=8)
    ax.plot(x4, y4, 'ro', markersize=8, label="End-Effector")

    # Annotations
    ax.text(x0, y0, f' ({x0:.2f}, {y0:.2f})', fontsize=10, va='bottom', ha='right')
    ax.text(x1, y1, f' ({x1:.2f}, {y1:.2f})', fontsize=10, va='bottom')
    ax.text(x2, y2, f' ({x2:.2f}, {y2:.2f})', fontsize=10, va='bottom')
    ax.text(x3, y3, f' ({x3:.2f}, {y3:.2f})', fontsize=10, va='bottom')
    ax.text(x4, y4, f' ({x4:.2f}, {y4:.2f})', fontsize=10, va='bottom', color='red')

    # Plot target if in IK mode
    if target is not None:
        tx, ty = target
        ax.plot(tx, ty, 'c*', markersize=14, label="Target")
        ax.text(tx, ty, f' Target: ({tx:.2f}, {ty:.2f})', fontsize=10, color='cyan', va='top')

    # Reach boundary circle
    max_len = l1 + l2 + l3 + l4
    reach_circle = plt.Circle((0, 0), max_len, color='cyan', fill=False, linestyle='--', alpha=0.4, label='Max Reach')
    ax.add_patch(reach_circle)

    ax.set_xlim([-max_len - 1, max_len + 1])
    ax.set_ylim([-max_len - 1, max_len + 1])
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc="upper left")
    plt.show()

if __name__ == "__main__":
    print("=" * 55)
    print("  4-DOF ROBOT ARM KINEMATICS (CLI & Matplotlib)")
    print("=" * 55)
    print("Select Mode:")
    print("  1. Forward Kinematics (FK) - Input joint angles -> get position")
    print("  2. Inverse Kinematics (IK) - Input target (X, Y) -> compute angles")
    mode = input("Enter choice (1 or 2, default 1): ").strip()

    l1 = float(input("\nEnter length of Link 1 (e.g. 5): ") or 5.0)
    l2 = float(input("Enter length of Link 2 (e.g. 4): ") or 4.0)
    l3 = float(input("Enter length of Link 3 (e.g. 3): ") or 3.0)
    l4 = float(input("Enter length of Link 4 (e.g. 2): ") or 2.0)

    if mode == '2':
        # Inverse Kinematics Mode
        tx = float(input("\nEnter Target X: "))
        ty = float(input("Enter Target Y: "))

        solved_angles, err, clamped = inverse_kinematics_ccd_4dof(tx, ty, l1, l2, l3, l4)
        t1, t2, t3, t4 = solved_angles
        print("\n--- IK Solution Found ---")
        if clamped:
            print("Notice: Target is outside workspace! Solved for closest reachable point.")
        print(f"Theta 1: {t1:.2f}°")
        print(f"Theta 2: {t2:.2f}°")
        print(f"Theta 3: {t3:.2f}°")
        print(f"Theta 4: {t4:.2f}°")
        print(f"Residual error: {err:.4f}")

        plot_arm(t1, t2, t3, t4, l1, l2, l3, l4, target=(tx, ty))

    else:
        # Forward Kinematics Mode
        t1 = float(input("\nEnter angle of Joint 1 (degrees): ") or 30.0)
        t2 = float(input("Enter angle of Joint 2 (degrees): ") or 45.0)
        t3 = float(input("Enter angle of Joint 3 (degrees): ") or -30.0)
        t4 = float(input("Enter angle of Joint 4 (degrees): ") or 20.0)

        x, y = forward_kinematics_4dof(np.radians(t1), np.radians(t2), np.radians(t3), np.radians(t4), l1, l2, l3, l4)
        print(f"\nEnd-Effector position: ({x:.2f}, {y:.2f})")
        print(f"Total orientation angle (phi): {t1 + t2 + t3 + t4:.2f}°")

        plot_arm(t1, t2, t3, t4, l1, l2, l3, l4)
