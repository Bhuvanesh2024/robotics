/**
 * Robot Kinematics Engine
 * Supports N-DOF planar arms (Default: 4-DOF matching user specification)
 * Algorithms:
 *  - Forward Kinematics (FK)
 *  - Inverse Kinematics (IK): CCD, FABRIK, Jacobian DLS (Damped Least Squares)
 *  - Manipulability & Singularity analysis
 *  - Trajectory interpolation
 */

class RobotKinematics {
    constructor(linkLengths = [5, 4, 3, 2], jointAngles = [30, 45, -30, 20]) {
        this.dof = linkLengths.length;
        this.links = [...linkLengths];
        // Angles stored in degrees
        this.angles = [...jointAngles];
        
        // Joint limits in degrees [min, max]
        this.limits = linkLengths.map(() => [-175, 175]);
        this.enableLimits = false;
        
        // Base position (x, y)
        this.base = { x: 0, y: 0 };
    }

    setDOF(num) {
        if (num < 1 || num > 6) return;
        const defaultLengths = [5, 4, 3, 2, 1.5, 1];
        const defaultAngles = [30, 45, -30, 20, 15, -15];
        
        const newLinks = [];
        const newAngles = [];
        const newLimits = [];
        
        for (let i = 0; i < num; i++) {
            newLinks.push(this.links[i] !== undefined ? this.links[i] : defaultLengths[i] || 3);
            newAngles.push(this.angles[i] !== undefined ? this.angles[i] : defaultAngles[i] || 0);
            newLimits.push(this.limits[i] || [-175, 175]);
        }
        
        this.dof = num;
        this.links = newLinks;
        this.angles = newAngles;
        this.limits = newLimits;
    }

    /**
     * Compute Forward Kinematics
     * Returns:
     * - positions: Array of {x, y} for each joint [Base, J1, J2, ..., EndEffector]
     * - cumulativeAngles: Array of absolute angles in radians for each link
     * - endEffector: { x, y, phi } (phi in radians and degrees)
     * - maxReach, minReach
     */
    forwardKinematics(customAngles = null, customLengths = null) {
        const anglesDeg = customAngles || this.angles;
        const lengths = customLengths || this.links;
        const dof = anglesDeg.length;

        const positions = [{ x: this.base.x, y: this.base.y }];
        const cumulativeAnglesRad = [];
        
        let currentAngleRad = 0;
        let currentX = this.base.x;
        let currentY = this.base.y;

        for (let i = 0; i < dof; i++) {
            const angleRad = (anglesDeg[i] * Math.PI) / 180;
            currentAngleRad += angleRad;
            cumulativeAnglesRad.push(currentAngleRad);

            currentX += lengths[i] * Math.cos(currentAngleRad);
            currentY += lengths[i] * Math.sin(currentAngleRad);
            positions.push({ x: currentX, y: currentY });
        }

        const endEffector = {
            x: currentX,
            y: currentY,
            phiRad: currentAngleRad,
            phiDeg: (currentAngleRad * 180) / Math.PI
        };

        const maxReach = lengths.reduce((acc, l) => acc + l, 0);
        // Minimum reach estimation
        const maxSingle = Math.max(...lengths);
        const sumOthers = maxReach - maxSingle;
        const minReach = Math.max(0, maxSingle - sumOthers);

        return {
            positions,
            cumulativeAnglesRad,
            endEffector,
            maxReach,
            minReach
        };
    }

    /**
     * Calculate 2D Jacobian Matrix for the current configuration
     * J is 2 x N (or 3 x N if orientation is considered)
     * J_v = [ - (y_ee - y_i-1) ]
     *       [   (x_ee - x_i-1) ]
     */
    computeJacobian(positions = null) {
        if (!positions) {
            positions = this.forwardKinematics().positions;
        }
        const ee = positions[positions.length - 1];
        const n = this.dof;
        
        // J is 2 x n (position only)
        const J = [
            new Array(n),
            new Array(n)
        ];

        for (let i = 0; i < n; i++) {
            const joint = positions[i]; // Joint pivot position (origin of link i+1)
            J[0][i] = -(ee.y - joint.y);
            J[1][i] = (ee.x - joint.x);
        }

        return J;
    }

    /**
     * Yoshikawa Manipulability Measure: w = sqrt(det(J * J^T))
     */
    computeManipulability(J = null) {
        if (!J) {
            J = this.computeJacobian();
        }
        // J * J^T is a 2x2 matrix: [ [a, b], [c, d] ]
        let a = 0, b = 0, c = 0, d = 0;
        const n = this.dof;
        for (let i = 0; i < n; i++) {
            a += J[0][i] * J[0][i];
            b += J[0][i] * J[1][i];
            c += J[1][i] * J[0][i];
            d += J[1][i] * J[1][i];
        }
        const det = a * d - b * c;
        return Math.sqrt(Math.max(0, det));
    }

    /**
     * Inverse Kinematics using FABRIK (Forward And Backward Reaching Inverse Kinematics)
     * Extremely fast, robust, and great for interactive dragging.
     */
    solveFABRIK(targetX, targetY, maxIterations = 30, tolerance = 0.001) {
        const lengths = [...this.links];
        const n = this.dof;
        const maxReach = lengths.reduce((sum, l) => sum + l, 0);

        const targetDist = Math.hypot(targetX - this.base.x, targetY - this.base.y);
        let reachable = true;

        // Target out of reach handling
        let tx = targetX;
        let ty = targetY;
        if (targetDist > maxReach) {
            reachable = false;
            const ratio = maxReach / (targetDist || 0.0001);
            tx = this.base.x + (targetX - this.base.x) * ratio;
            ty = this.base.y + (targetY - this.base.y) * ratio;
        }

        // Get initial joint positions via FK
        let pts = this.forwardKinematics().positions.map(p => ({ ...p }));

        let iter = 0;
        let diff = Math.hypot(pts[n].x - tx, pts[n].y - ty);

        while (diff > tolerance && iter < maxIterations) {
            // Stage 1: Forward Reaching (from end to base)
            pts[n] = { x: tx, y: ty };
            for (let i = n - 1; i >= 0; i--) {
                const dist = Math.hypot(pts[i + 1].x - pts[i].x, pts[i + 1].y - pts[i].y);
                const lambda = lengths[i] / (dist || 0.0001);
                pts[i] = {
                    x: (1 - lambda) * pts[i + 1].x + lambda * pts[i].x,
                    y: (1 - lambda) * pts[i + 1].y + lambda * pts[i].y
                };
            }

            // Stage 2: Backward Reaching (from base to end)
            pts[0] = { x: this.base.x, y: this.base.y };
            for (let i = 0; i < n; i++) {
                const dist = Math.hypot(pts[i + 1].x - pts[i].x, pts[i + 1].y - pts[i].y);
                const lambda = lengths[i] / (dist || 0.0001);
                pts[i + 1] = {
                    x: (1 - lambda) * pts[i].x + lambda * pts[i + 1].x,
                    y: (1 - lambda) * pts[i].y + lambda * pts[i + 1].y
                };
            }

            diff = Math.hypot(pts[n].x - tx, pts[n].y - ty);
            iter++;
        }

        // Convert positions back to relative joint angles
        const solvedAngles = [];
        let prevGlobalAngle = 0;

        for (let i = 0; i < n; i++) {
            const dx = pts[i + 1].x - pts[i].x;
            const dy = pts[i + 1].y - pts[i].y;
            const globalAngle = Math.atan2(dy, dx);
            let relAngle = globalAngle - prevGlobalAngle;
            
            // Normalize to [-pi, pi]
            while (relAngle > Math.PI) relAngle -= 2 * Math.PI;
            while (relAngle < -Math.PI) relAngle += 2 * Math.PI;

            let deg = (relAngle * 180) / Math.PI;
            if (this.enableLimits) {
                deg = Math.max(this.limits[i][0], Math.min(this.limits[i][1], deg));
            }
            solvedAngles.push(deg);
            prevGlobalAngle = (prevGlobalAngle * 180 / Math.PI + deg) * Math.PI / 180;
        }

        this.angles = solvedAngles;
        return {
            converged: diff <= tolerance,
            iterations: iter,
            error: diff,
            reachable
        };
    }

    /**
     * Inverse Kinematics using CCD (Cyclic Coordinate Descent)
     * Iterates backwards from end joint to base, rotating each joint towards the target.
     */
    solveCCD(targetX, targetY, maxIterations = 35, tolerance = 0.005) {
        const n = this.dof;
        const maxReach = this.links.reduce((s, l) => s + l, 0);
        const targetDist = Math.hypot(targetX - this.base.x, targetY - this.base.y);
        let reachable = targetDist <= maxReach;

        let iter = 0;
        let currentError = Infinity;

        while (iter < maxIterations) {
            let fk = this.forwardKinematics();
            let ee = fk.positions[n];
            currentError = Math.hypot(targetX - ee.x, targetY - ee.y);
            if (currentError <= tolerance) break;

            // Iterate backwards through joints
            for (let i = n - 1; i >= 0; i--) {
                fk = this.forwardKinematics();
                const jointPos = fk.positions[i];
                ee = fk.positions[n];

                // Vector from current joint to end-effector
                const toEE = { x: ee.x - jointPos.x, y: ee.y - jointPos.y };
                // Vector from current joint to target
                const toTarget = { x: targetX - jointPos.x, y: targetY - jointPos.y };

                const magEE = Math.hypot(toEE.x, toEE.y);
                const magTarget = Math.hypot(toTarget.x, toTarget.y);

                if (magEE > 1e-6 && magTarget > 1e-6) {
                    // Cross product and dot product to find signed angle
                    const cross = toEE.x * toTarget.y - toEE.y * toTarget.x;
                    const dot = toEE.x * toTarget.x + toEE.y * toTarget.y;
                    let angleDelta = Math.atan2(cross, dot); // in radians

                    // Damping factor for smooth convergence
                    const damping = 0.85;
                    let deltaDeg = (angleDelta * 180 / Math.PI) * damping;

                    let newAngle = this.angles[i] + deltaDeg;
                    // Normalize to [-180, 180]
                    while (newAngle > 180) newAngle -= 360;
                    while (newAngle < -180) newAngle += 360;

                    if (this.enableLimits) {
                        newAngle = Math.max(this.limits[i][0], Math.min(this.limits[i][1], newAngle));
                    }
                    this.angles[i] = newAngle;
                }
            }
            iter++;
        }

        const finalFK = this.forwardKinematics();
        const finalEE = finalFK.positions[n];
        const finalError = Math.hypot(targetX - finalEE.x, targetY - finalEE.y);

        return {
            converged: finalError <= tolerance,
            iterations: iter,
            error: finalError,
            reachable
        };
    }

    /**
     * Inverse Kinematics using Jacobian DLS (Damped Least Squares / Levenberg-Marquardt)
     * Classic robotics formulation:
     * DeltaTheta = J^T * (J * J^T + lambda^2 * I)^(-1) * DeltaX
     */
    solveJacobianDLS(targetX, targetY, targetPhiDeg = null, maxIterations = 40, tolerance = 0.005) {
        const n = this.dof;
        const lambda = 0.4; // Damping parameter
        const stepSize = 0.5;
        let iter = 0;
        let currentError = Infinity;

        const constrainPhi = targetPhiDeg !== null;

        while (iter < maxIterations) {
            const fk = this.forwardKinematics();
            const ee = fk.endEffector;
            const dx = targetX - ee.x;
            const dy = targetY - ee.y;
            currentError = Math.hypot(dx, dy);

            if (currentError <= tolerance && (!constrainPhi || Math.abs(targetPhiDeg - ee.phiDeg) < 0.5)) {
                break;
            }

            // Error vector
            const errorVec = constrainPhi 
                ? [dx, dy, ((targetPhiDeg - ee.phiDeg) * Math.PI / 180)]
                : [dx, dy];

            const rows = constrainPhi ? 3 : 2;

            // Construct Jacobian
            const J = [];
            for (let r = 0; r < rows; r++) J.push(new Array(n));

            for (let i = 0; i < n; i++) {
                const joint = fk.positions[i];
                J[0][i] = -(ee.y - joint.y);
                J[1][i] = (ee.x - joint.x);
                if (constrainPhi) {
                    J[2][i] = 1.0; // derivative of phi wrt theta_i is 1
                }
            }

            // Compute A = J * J^T + lambda^2 * I (rows x rows)
            const A = Array.from({ length: rows }, () => new Array(rows).fill(0));
            for (let r1 = 0; r1 < rows; r1++) {
                for (let r2 = 0; r2 < rows; r2++) {
                    let sum = 0;
                    for (let c = 0; c < n; c++) {
                        sum += J[r1][c] * J[r2][c];
                    }
                    A[r1][r2] = sum + (r1 === r2 ? lambda * lambda : 0);
                }
            }

            // Invert A (2x2 or 3x3)
            let A_inv;
            if (rows === 2) {
                const det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
                if (Math.abs(det) < 1e-7) break;
                const invDet = 1 / det;
                A_inv = [
                    [A[1][1] * invDet, -A[0][1] * invDet],
                    [-A[1][0] * invDet, A[0][0] * invDet]
                ];
            } else {
                // 3x3 inversion
                A_inv = invertMatrix3x3(A);
                if (!A_inv) break;
            }

            // v = A_inv * errorVec
            const v = new Array(rows).fill(0);
            for (let r = 0; r < rows; r++) {
                for (let c = 0; c < rows; c++) {
                    v[r] += A_inv[r][c] * errorVec[c];
                }
            }

            // deltaTheta = J^T * v
            for (let i = 0; i < n; i++) {
                let dTheta = 0;
                for (let r = 0; r < rows; r++) {
                    dTheta += J[r][i] * v[r];
                }
                let degDelta = (dTheta * 180 / Math.PI) * stepSize;
                degDelta = Math.max(-25, Math.min(25, degDelta)); // clamp step
                let newAngle = this.angles[i] + degDelta;
                while (newAngle > 180) newAngle -= 360;
                while (newAngle < -180) newAngle += 360;

                if (this.enableLimits) {
                    newAngle = Math.max(this.limits[i][0], Math.min(this.limits[i][1], newAngle));
                }
                this.angles[i] = newAngle;
            }

            iter++;
        }

        const finalFK = this.forwardKinematics();
        const finalEE = finalFK.endEffector;
        const finalError = Math.hypot(targetX - finalEE.x, targetY - finalEE.y);

        return {
            converged: finalError <= tolerance,
            iterations: iter,
            error: finalError,
            reachable: currentError <= tolerance
        };
    }
}

// 3x3 Matrix Inverter Helper
function invertMatrix3x3(m) {
    const a = m[0][0], b = m[0][1], c = m[0][2];
    const d = m[1][0], e = m[1][1], f = m[1][2];
    const g = m[2][0], h = m[2][1], k = m[2][2];

    const A = (e * k - f * h);
    const B = -(d * k - f * g);
    const C = (d * h - e * g);
    const D = -(b * k - c * h);
    const E = (a * k - c * g);
    const F = -(a * h - b * g);
    const G = (b * f - c * e);
    const H = -(a * f - c * d);
    const K = (a * e - b * d);

    const det = a * A + b * B + c * C;
    if (Math.abs(det) < 1e-8) return null;

    const invDet = 1 / det;
    return [
        [A * invDet, D * invDet, G * invDet],
        [B * invDet, E * invDet, H * invDet],
        [C * invDet, F * invDet, K * invDet]
    ];
}

// Export for module or browser window
if (typeof window !== 'undefined') {
    window.RobotKinematics = RobotKinematics;
}
