/**
 * Main Application Controller for Robot Kinematics Visualizer (3D & 2D)
 * Integrates:
 *  - 2D Canvas Visualizer (RobotCanvas)
 *  - 3D Three.js Visualizer (Robot3DVisualizer)
 *  - Seamless 2D <-> 3D Synchronization
 *  - Forward Kinematics (FK) and Inverse Kinematics (IK) Solvers
 *  - 3D Autonomous Trajectory Tracking & Gripper Controls
 *  - Python Code Exporter (2D & 3D Matplotlib)
 */

document.addEventListener('DOMContentLoaded', () => {
    // Initial Robot Arm (4-DOF default: Base Yaw, Shoulder Pitch, Elbow Pitch, Wrist Pitch)
    const robot = new RobotKinematics([5, 4, 3, 2], [30, 45, -30, 20]);

    // View elements
    const canvas2DEl = document.getElementById('robotCanvas');
    const container3DEl = document.getElementById('robot3DContainer');

    // Initialize 2D and 3D Visualizers
    const visualizer2D = new RobotCanvas(canvas2DEl, robot);
    const visualizer3D = new Robot3DVisualizer(container3DEl, robot);
    visualizer3D.animate();

    // Application state
    const state = {
        view: '3d', // '3d' | '2d'
        mode: 'fk', // 'fk' | 'ik' | 'trajectory'
        ikSolver: 'fabrik', // 'fabrik' | 'ccd' | 'jacobian'
        constrainOrientation: false,
        targetOrientation: 0,
        isPlayingTrajectory: false,
        trajectoryShape: 'circle',
        trajectorySpeed: 1,
        trajectoryProgress: 0,
        trajectoryTimer: null
    };

    // Initialize UI controls
    buildJointSliders();
    updateTelemetry();

    // Set initial 3D View
    setViewMode('3d');

    // Theme Management
    const themeSelect = document.getElementById('themeSelect');
    const savedTheme = localStorage.getItem('robokinematics_theme') || 'industrial';

    function applyTheme(theme) {
        document.documentElement.setAttribute('data-theme', theme);
        document.body.setAttribute('data-theme', theme);
        if (themeSelect) themeSelect.value = theme;
        visualizer3D.setTheme(theme);
        visualizer2D.setTheme(theme);
        localStorage.setItem('robokinematics_theme', theme);
    }

    if (themeSelect) {
        themeSelect.addEventListener('change', (e) => {
            applyTheme(e.target.value);
            showToast(`Theme switched to ${e.target.options[e.target.selectedIndex].text}`);
        });
    }

    applyTheme(savedTheme);

    // 2D / 3D Mode Switcher
    const btnMode2D = document.getElementById('btnMode2D');
    const btnMode3D = document.getElementById('btnMode3D');

    function setViewMode(mode) {
        state.view = mode;
        const thZ = document.getElementById('thZ');
        const grpZ = document.getElementById('groupTargetZ');
        const camBar = document.getElementById('cameraPresetBar');
        const badge = document.getElementById('viewModeBadge');
        const tipText = document.getElementById('bottomTipText');
        const lblGizmo = document.getElementById('lblToggleGizmo');

        if (mode === '3d') {
            btnMode3D?.classList.add('active');
            btnMode2D?.classList.remove('active');
            container3DEl.classList.add('active');
            canvas2DEl.classList.add('hidden');

            if (thZ) thZ.style.display = '';
            if (grpZ) grpZ.style.display = '';
            if (camBar) camBar.style.display = 'flex';
            if (lblGizmo) lblGizmo.style.display = '';
            if (badge) badge.textContent = '3D Robotics Lab';
            if (tipText) tipText.innerHTML = '<strong>3D Controls:</strong> Grab the <strong>3D Gizmo arrows</strong> to solve IK in 3D! Left-drag background to orbit, right-drag to pan, wheel to zoom.';

            visualizer3D.resize();
            visualizer3D.updateRobotPose();
        } else {
            btnMode2D?.classList.add('active');
            btnMode3D?.classList.remove('active');
            container3DEl.classList.remove('active');
            canvas2DEl.classList.remove('hidden');

            if (thZ) thZ.style.display = 'none';
            if (grpZ) grpZ.style.display = 'none';
            if (camBar) camBar.style.display = 'none';
            if (lblGizmo) lblGizmo.style.display = 'none';
            if (badge) badge.textContent = '2D Planar Mode';
            if (tipText) tipText.innerHTML = '<strong>2D Controls:</strong> Click & drag the <strong>Target crosshair</strong> or <strong>End-Effector (red dot)</strong> to solve IK! Mouse wheel to zoom.';

            visualizer2D.resize();
            visualizer2D.fitToScreen();
            visualizer2D.render();
        }
        updateTelemetry();
    }

    btnMode2D?.addEventListener('click', () => setViewMode('2d'));
    btnMode3D?.addEventListener('click', () => setViewMode('3d'));

    // Camera preset buttons
    document.querySelectorAll('.btn-cam').forEach(btn => {
        btn.addEventListener('click', () => {
            const cam = btn.dataset.cam;
            visualizer3D.setCameraPreset(cam);
        });
    });

    // 3D Target Dragging callback from TransformControls
    visualizer3D.onTargetChange = (tx, ty, tz) => {
        const inpX = document.getElementById('inputTargetX');
        const inpY = document.getElementById('inputTargetY');
        const inpZ = document.getElementById('inputTargetZ');
        if (inpX) inpX.value = tx.toFixed(2);
        if (inpY) inpY.value = ty.toFixed(2);
        if (inpZ) inpZ.value = tz.toFixed(2);

        visualizer2D.target.x = tx;
        visualizer2D.target.y = ty;
        updateJointSliders();
        updateTelemetry();
        visualizer2D.render();
    };

    visualizer3D.onAnglesChange = () => {
        updateJointSliders();
        updateTelemetry();
        visualizer2D.render();
    };

    // 2D Target Dragging callback
    visualizer2D.onTargetChange = (tx, ty) => {
        const inpX = document.getElementById('inputTargetX');
        const inpY = document.getElementById('inputTargetY');
        if (inpX) inpX.value = tx.toFixed(2);
        if (inpY) inpY.value = ty.toFixed(2);

        if (state.view === '3d') {
            const tz = parseFloat(document.getElementById('inputTargetZ')?.value) || 0;
            visualizer3D.target.set(tx, ty, tz);
            visualizer3D.targetMesh.position.set(tx, ty, tz);
            visualizer3D.solve3DIK(tx, ty, tz);
        } else {
            solveIK(tx, ty);
        }

        updateJointSliders();
        updateTelemetry();
        visualizer3D.updateRobotPose();
    };

    // Joint angle dragging in 2D
    visualizer2D.onJointAngleChange = (jointIndex, deg) => {
        robot.angles[jointIndex] = deg;
        updateJointSliders();
        updateTelemetry();
        visualizer2D.render();
        visualizer3D.updateRobotPose();
    };

    // Gripper Slider
    const gripperSlider = document.getElementById('gripperSlider');
    if (gripperSlider) {
        gripperSlider.addEventListener('input', (e) => {
            const val = parseFloat(e.target.value);
            visualizer3D.setGripper(val);
            const badge = document.getElementById('gripperValBadge');
            const statusText = document.getElementById('gripperStatusText');
            const pct = Math.round(val * 100);
            if (badge) badge.textContent = `${pct}% Open`;
            if (statusText) statusText.textContent = val < 0.1 ? 'Closed' : (val > 0.8 ? 'Open' : 'Grip');
        });
    }

    // Mode Switcher Tabs
    const modeTabs = document.querySelectorAll('.mode-tab');
    modeTabs.forEach(tab => {
        tab.addEventListener('click', () => {
            modeTabs.forEach(t => t.classList.remove('active'));
            tab.classList.add('active');
            state.mode = tab.dataset.mode;

            document.querySelectorAll('.mode-panel').forEach(p => p.classList.add('hidden'));
            document.getElementById(`panel-${state.mode}`).classList.remove('hidden');

            if (state.mode === 'trajectory') {
                visualizer2D.clearTrail();
                visualizer3D.clearTrail();
            }
        });
    });

    // Arm DOF Selector
    const dofSelect = document.getElementById('dofSelect');
    if (dofSelect) {
        dofSelect.addEventListener('change', (e) => {
            const dof = parseInt(e.target.value, 10);
            robot.setDOF(dof);
            buildJointSliders();
            visualizer3D.buildRobot();
            visualizer3D.updateRobotPose();
            visualizer3D.updateWorkspaceSphere();
            updateTelemetry();
            visualizer2D.fitToScreen();
            generatePythonCode();
        });
    }

    // IK Solver Select
    const ikSolverSelect = document.getElementById('ikSolverSelect');
    if (ikSolverSelect) {
        ikSolverSelect.addEventListener('change', (e) => {
            state.ikSolver = e.target.value;
            triggerIKSolve();
        });
    }

    // Target Inputs in IK Panel
    const inputTargetX = document.getElementById('inputTargetX');
    const inputTargetY = document.getElementById('inputTargetY');
    const inputTargetZ = document.getElementById('inputTargetZ');
    const btnSolveIK = document.getElementById('btnSolveIK');

    function triggerIKSolve() {
        const tx = parseFloat(inputTargetX?.value) || 0;
        const ty = parseFloat(inputTargetY?.value) || 0;
        const tz = parseFloat(inputTargetZ?.value) || 0;

        if (state.view === '3d') {
            visualizer3D.target.set(tx, ty, tz);
            visualizer3D.targetMesh.position.set(tx, ty, tz);
            visualizer3D.solve3DIK(tx, ty, tz);
        } else {
            visualizer2D.target.x = tx;
            visualizer2D.target.y = ty;
            solveIK(tx, ty);
        }

        updateJointSliders();
        updateTelemetry();
        visualizer2D.render();
        visualizer3D.updateRobotPose();
    }

    inputTargetX?.addEventListener('input', triggerIKSolve);
    inputTargetY?.addEventListener('input', triggerIKSolve);
    inputTargetZ?.addEventListener('input', triggerIKSolve);
    btnSolveIK?.addEventListener('click', triggerIKSolve);

    // Orientation constraint toggle
    const chkConstrainPhi = document.getElementById('chkConstrainPhi');
    const inputTargetPhi = document.getElementById('inputTargetPhi');
    if (chkConstrainPhi) {
        chkConstrainPhi.addEventListener('change', (e) => {
            state.constrainOrientation = e.target.checked;
            if (inputTargetPhi) inputTargetPhi.disabled = !state.constrainOrientation;
            triggerIKSolve();
        });
    }
    if (inputTargetPhi) {
        inputTargetPhi.addEventListener('input', (e) => {
            state.targetOrientation = parseFloat(e.target.value) || 0;
            triggerIKSolve();
        });
    }

    // Joint Limits Toggle
    document.getElementById('chkLimits')?.addEventListener('change', (e) => {
        robot.enableLimits = e.target.checked;
        updateTelemetry();
        visualizer2D.render();
        visualizer3D.updateRobotPose();
    });

    // Display Toggles
    document.getElementById('toggleGrid')?.addEventListener('change', (e) => {
        visualizer2D.showGrid = e.target.checked;
        visualizer2D.render();
    });
    document.getElementById('toggleWorkspace')?.addEventListener('change', (e) => {
        visualizer2D.showWorkspace = e.target.checked;
        visualizer3D.showWorkspace = e.target.checked;
        visualizer3D.updateWorkspaceSphere();
        visualizer2D.render();
    });
    document.getElementById('toggleTrail')?.addEventListener('change', (e) => {
        visualizer2D.showTrail = e.target.checked;
        visualizer3D.showTrail = e.target.checked;
        if (!e.target.checked) {
            visualizer2D.clearTrail();
            visualizer3D.clearTrail();
        }
        visualizer2D.render();
    });
    document.getElementById('toggleGizmo')?.addEventListener('change', (e) => {
        if (visualizer3D.transformGizmo) {
            visualizer3D.transformGizmo.visible = e.target.checked;
            visualizer3D.transformGizmo.enabled = e.target.checked;
        }
    });
    document.getElementById('toggleLabels')?.addEventListener('change', (e) => {
        visualizer2D.showLabels = e.target.checked;
        visualizer2D.render();
    });

    // Viewport reset actions
    document.getElementById('btnFitScreen')?.addEventListener('click', () => {
        if (state.view === '3d') {
            visualizer3D.setCameraPreset('iso');
        } else {
            visualizer2D.fitToScreen();
        }
    });
    document.getElementById('btnResetOrigin')?.addEventListener('click', () => {
        if (state.view === '3d') {
            visualizer3D.controls.target.set(0, 4, 0);
            visualizer3D.controls.update();
        } else {
            visualizer2D.pan.x = visualizer2D.width / 2;
            visualizer2D.pan.y = visualizer2D.height / 2;
            visualizer2D.render();
        }
    });
    document.getElementById('btnClearTrail')?.addEventListener('click', () => {
        visualizer2D.clearTrail();
        visualizer3D.clearTrail();
    });

    // Preset Arm Poses
    document.querySelectorAll('.btn-pose').forEach(btn => {
        btn.addEventListener('click', () => {
            applyPose(btn.dataset.pose);
        });
    });

    // Trajectory Controls
    const shapeSelect = document.getElementById('trajectoryShape');
    const btnPlayTrajectory = document.getElementById('btnPlayTrajectory');
    const btnStopTrajectory = document.getElementById('btnStopTrajectory');
    const speedRange = document.getElementById('trajectorySpeed');

    if (shapeSelect) {
        shapeSelect.addEventListener('change', (e) => {
            state.trajectoryShape = e.target.value;
            visualizer2D.clearTrail();
            visualizer3D.clearTrail();
        });
    }
    if (speedRange) {
        speedRange.addEventListener('input', (e) => {
            state.trajectorySpeed = parseFloat(e.target.value);
            document.getElementById('speedVal').textContent = `${state.trajectorySpeed.toFixed(1)}x`;
        });
    }
    if (btnPlayTrajectory) {
        btnPlayTrajectory.addEventListener('click', () => {
            if (state.isPlayingTrajectory) {
                stopTrajectory();
            } else {
                startTrajectory();
            }
        });
    }
    if (btnStopTrajectory) {
        btnStopTrajectory.addEventListener('click', () => stopTrajectory());
    }

    // Code export copy & download
    document.getElementById('btnCopyPython')?.addEventListener('click', () => {
        const code = document.getElementById('pythonCodeOutput').value;
        navigator.clipboard.writeText(code).then(() => {
            showToast('Python code copied to clipboard!');
        });
    });

    document.getElementById('btnDownloadPython')?.addEventListener('click', () => {
        const code = document.getElementById('pythonCodeOutput').value;
        const blob = new Blob([code], { type: 'text/x-python' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `robot_arm_${robot.dof}dof_kinematics.py`;
        a.click();
        URL.revokeObjectURL(url);
    });

    /**
     * Build Dynamic Sliders for Link Lengths and Joint Angles
     */
    function buildJointSliders() {
        const container = document.getElementById('jointSlidersContainer');
        if (!container) return;
        container.innerHTML = '';

        for (let i = 0; i < robot.dof; i++) {
            const color = visualizer2D.linkColors[i % visualizer2D.linkColors.length];
            const card = document.createElement('div');
            card.className = 'joint-control-card';
            card.style.borderLeft = `4px solid ${color}`;

            const jointName = (i === 0 && state.view === '3d')
                ? 'Joint 1 (Base Yaw)'
                : (i === 1 && state.view === '3d')
                ? 'Joint 2 (Shoulder)'
                : `Joint ${i + 1}`;

            card.innerHTML = `
                <div class="card-header">
                    <div class="joint-title">
                        <span class="color-dot" style="background:${color}"></span>
                        <strong>${jointName} & Link ${i + 1}</strong>
                    </div>
                    <span class="angle-badge" id="badge-theta-${i}">${robot.angles[i].toFixed(1)}°</span>
                </div>

                <div class="slider-row">
                    <label>Angle θ${i + 1}</label>
                    <input type="range" class="joint-angle-slider" min="-180" max="180" step="0.5" value="${robot.angles[i]}" data-joint="${i}">
                    <div class="input-with-unit">
                        <input type="number" class="joint-angle-num" min="-360" max="360" step="1" value="${robot.angles[i].toFixed(1)}" data-joint="${i}">
                        <span>°</span>
                    </div>
                </div>

                <div class="slider-row">
                    <label>Length L${i + 1}</label>
                    <input type="range" class="joint-length-slider" min="0.5" max="12" step="0.1" value="${robot.links[i]}" data-link="${i}">
                    <div class="input-with-unit">
                        <input type="number" class="joint-length-num" min="0.1" max="25" step="0.1" value="${robot.links[i].toFixed(1)}" data-link="${i}">
                        <span>m</span>
                    </div>
                </div>
            `;

            container.appendChild(card);
        }

        container.querySelectorAll('.joint-angle-slider').forEach(slider => {
            slider.addEventListener('input', (e) => {
                const idx = parseInt(e.target.dataset.joint, 10);
                const val = parseFloat(e.target.value);
                robot.angles[idx] = val;
                syncInputs(idx, 'angle', val);
                updateTelemetry();
                visualizer2D.render();
                visualizer3D.updateRobotPose();
            });
        });

        container.querySelectorAll('.joint-angle-num').forEach(input => {
            input.addEventListener('input', (e) => {
                const idx = parseInt(e.target.dataset.joint, 10);
                const val = parseFloat(e.target.value) || 0;
                robot.angles[idx] = val;
                syncInputs(idx, 'angle', val);
                updateTelemetry();
                visualizer2D.render();
                visualizer3D.updateRobotPose();
            });
        });

        container.querySelectorAll('.joint-length-slider').forEach(slider => {
            slider.addEventListener('input', (e) => {
                const idx = parseInt(e.target.dataset.link, 10);
                const val = parseFloat(e.target.value);
                robot.links[idx] = val;
                syncInputs(idx, 'length', val);
                visualizer3D.buildRobot();
                visualizer3D.updateRobotPose();
                visualizer3D.updateWorkspaceSphere();
                updateTelemetry();
                visualizer2D.render();
            });
        });

        container.querySelectorAll('.joint-length-num').forEach(input => {
            input.addEventListener('input', (e) => {
                const idx = parseInt(e.target.dataset.link, 10);
                const val = parseFloat(e.target.value) || 1;
                robot.links[idx] = val;
                syncInputs(idx, 'length', val);
                visualizer3D.buildRobot();
                visualizer3D.updateRobotPose();
                visualizer3D.updateWorkspaceSphere();
                updateTelemetry();
                visualizer2D.render();
            });
        });
    }

    function syncInputs(idx, type, val) {
        if (type === 'angle') {
            const badge = document.getElementById(`badge-theta-${idx}`);
            if (badge) badge.textContent = `${val.toFixed(1)}°`;
            const slider = document.querySelector(`.joint-angle-slider[data-joint="${idx}"]`);
            const num = document.querySelector(`.joint-angle-num[data-joint="${idx}"]`);
            if (slider && parseFloat(slider.value) !== val) slider.value = val;
            if (num && parseFloat(num.value) !== val) num.value = val.toFixed(1);
        } else if (type === 'length') {
            const slider = document.querySelector(`.joint-length-slider[data-link="${idx}"]`);
            const num = document.querySelector(`.joint-length-num[data-link="${idx}"]`);
            if (slider && parseFloat(slider.value) !== val) slider.value = val;
            if (num && parseFloat(num.value) !== val) num.value = val.toFixed(1);
        }
    }

    function updateJointSliders() {
        for (let i = 0; i < robot.dof; i++) {
            syncInputs(i, 'angle', robot.angles[i]);
            syncInputs(i, 'length', robot.links[i]);
        }
    }

    /**
     * Solve 2D Inverse Kinematics
     */
    function solveIK(tx, ty) {
        let result;
        const phiDeg = state.constrainOrientation ? state.targetOrientation : null;

        if (state.ikSolver === 'fabrik') {
            result = robot.solveFABRIK(tx, ty);
        } else if (state.ikSolver === 'ccd') {
            result = robot.solveCCD(tx, ty);
        } else if (state.ikSolver === 'jacobian') {
            result = robot.solveJacobianDLS(tx, ty, phiDeg);
        }

        updateJointSliders();
        updateTelemetry(result);
        visualizer2D.render();
    }

    /**
     * Update Telemetry HUD, Matrix Display, and Math Cards
     */
    function updateTelemetry(ikResult = null) {
        const fk = robot.forwardKinematics();
        const ee = fk.endEffector;
        const maxReach = fk.maxReach;

        // End-Effector Readouts (2D or 3D)
        const elEEPos = document.getElementById('telemetryEEPos');
        if (elEEPos) {
            if (state.view === '3d' && visualizer3D.eeMountNode) {
                const ee3D = new THREE.Vector3();
                visualizer3D.eeMountNode.getWorldPosition(ee3D);
                elEEPos.textContent = `(${ee3D.x.toFixed(2)}, ${ee3D.y.toFixed(2)}, ${ee3D.z.toFixed(2)})`;
            } else {
                elEEPos.textContent = `(${ee.x.toFixed(2)}, ${ee.y.toFixed(2)})`;
            }
        }

        const elEEPhi = document.getElementById('telemetryEEPhi');
        if (elEEPhi) elEEPhi.textContent = `${ee.phiDeg.toFixed(1)}° (${ee.phiRad.toFixed(2)} rad)`;

        const elMaxReach = document.getElementById('telemetryMaxReach');
        if (elMaxReach) elMaxReach.textContent = `${maxReach.toFixed(2)} m`;

        // Reachability Status
        const targetDist = state.view === '3d'
            ? visualizer3D.target.length()
            : Math.hypot(visualizer2D.target.x, visualizer2D.target.y);

        const elStatus = document.getElementById('reachabilityStatus');
        if (elStatus) {
            if (targetDist > maxReach + 1.5) {
                elStatus.className = 'status-pill out-of-reach';
                elStatus.textContent = 'Out of Reach (Clamped)';
            } else {
                elStatus.className = 'status-pill reachable';
                elStatus.textContent = 'Reachable / Solved';
            }
        }

        // Coordinates Table
        const coordsBody = document.getElementById('coordsTableBody');
        if (coordsBody) {
            let html = '';
            for (let i = 0; i < fk.positions.length; i++) {
                const p = fk.positions[i];
                const isEE = (i === fk.positions.length - 1);
                const label = i === 0 ? 'Base (J0)' : (isEE ? `End-Effector (J${i})` : `Joint ${i}`);
                const color = i === 0 ? '#94a3b8' : visualizer2D.linkColors[(i - 1) % visualizer2D.linkColors.length];

                let zCol = '';
                if (state.view === '3d') {
                    // Compute 3D coords
                    const yawRad = (robot.angles[0] * Math.PI) / 180;
                    const r = p.x;
                    const x3D = r * Math.cos(yawRad);
                    const z3D = r * Math.sin(yawRad);
                    const y3D = p.y + visualizer3D.baseHeight;
                    zCol = `<td class="mono font-bold">${z3D.toFixed(2)}</td>`;
                    html += `
                        <tr>
                            <td><span class="color-dot" style="background:${color}"></span> ${label}</td>
                            <td class="mono font-bold">${x3D.toFixed(2)}</td>
                            <td class="mono font-bold">${y3D.toFixed(2)}</td>
                            ${zCol}
                        </tr>
                    `;
                } else {
                    html += `
                        <tr>
                            <td><span class="color-dot" style="background:${color}"></span> ${label}</td>
                            <td class="mono font-bold">${p.x.toFixed(2)}</td>
                            <td class="mono font-bold">${p.y.toFixed(2)}</td>
                        </tr>
                    `;
                }
            }
            coordsBody.innerHTML = html;
        }

        // Live Jacobian & Manipulability
        const J = robot.computeJacobian(fk.positions);
        const manip = robot.computeManipulability(J);

        const elManip = document.getElementById('telemetryManipulability');
        if (elManip) elManip.textContent = manip.toFixed(3);

        const elJacobianMatrix = document.getElementById('jacobianMatrixOutput');
        if (elJacobianMatrix) {
            let jStr = `J (2×${robot.dof}) =\n`;
            jStr += `[ ` + J[0].map(v => (v >= 0 ? ' ' : '') + v.toFixed(2)).join(', ') + ` ]\n`;
            jStr += `[ ` + J[1].map(v => (v >= 0 ? ' ' : '') + v.toFixed(2)).join(', ') + ` ]`;
            elJacobianMatrix.textContent = jStr;
        }

        // Generate matching Python code
        generatePythonCode();
    }

    /**
     * Preset Arm Configurations
     */
    function applyPose(pose) {
        if (pose === 'zero') {
            robot.angles = robot.angles.map(() => 0);
        } else if (pose === 'home') {
            robot.angles = [30, 45, -30, 20, 15, -15].slice(0, robot.dof);
        } else if (pose === 'upright') {
            robot.angles = [0, 90, 0, 0, 0, 0].slice(0, robot.dof);
        } else if (pose === 'folded') {
            robot.angles = [0, 135, -150, 150, -150, 150].slice(0, robot.dof);
        } else if (pose === 'reach') {
            robot.angles = [0, 10, 10, 10, 5, 5].slice(0, robot.dof);
        }

        updateJointSliders();
        const fk = robot.forwardKinematics();
        visualizer2D.target.x = fk.endEffector.x;
        visualizer2D.target.y = fk.endEffector.y;

        if (visualizer3D.eeMountNode) {
            visualizer3D.updateRobotPose();
            const eeWorld = new THREE.Vector3();
            visualizer3D.eeMountNode.getWorldPosition(eeWorld);
            visualizer3D.target.copy(eeWorld);
            visualizer3D.targetMesh.position.copy(eeWorld);
        }

        updateTelemetry();
        visualizer2D.render();
    }

    /**
     * 3D & 2D Trajectory Player: smoothly traces paths in real-time
     */
    function startTrajectory() {
        state.isPlayingTrajectory = true;
        const btnPlay = document.getElementById('btnPlayTrajectory');
        if (btnPlay) {
            btnPlay.innerHTML = `
                <svg width="14" height="14" viewBox="0 0 24 24" fill="currentColor"><rect x="6" y="4" width="4" height="16"/><rect x="14" y="4" width="4" height="16"/></svg>
                Pause Trajectory
            `;
            btnPlay.classList.add('playing');
        }

        let lastTime = performance.now();

        function animate(now) {
            if (!state.isPlayingTrajectory) return;

            const dt = (now - lastTime) / 1000;
            lastTime = now;

            state.trajectoryProgress += dt * 0.4 * state.trajectorySpeed;
            if (state.trajectoryProgress > Math.PI * 2) {
                state.trajectoryProgress -= Math.PI * 2;
            }

            const t = state.trajectoryProgress;
            const maxReach = robot.links.reduce((a, b) => a + b, 0);
            const r = maxReach * 0.35;
            const cx = maxReach * 0.45;
            const cy = maxReach * 0.25;

            let tx, ty, tz;

            if (state.view === '3d') {
                if (state.trajectoryShape === 'circle') {
                    tx = r * Math.cos(t) + 4;
                    ty = 4 + r * 0.4 * Math.sin(t);
                    tz = r * Math.sin(t) + 2;
                } else if (state.trajectoryShape === 'helix') {
                    tx = r * Math.cos(t * 2) + 3;
                    ty = 2 + (t / (Math.PI * 2)) * 6;
                    tz = r * Math.sin(t * 2) + 3;
                } else if (state.trajectoryShape === 'lemniscate') {
                    tx = 5 + r * Math.sin(t);
                    ty = 4 + (r * 0.5) * Math.sin(2 * t);
                    tz = 3 + (r * 0.6) * Math.cos(t);
                } else if (state.trajectoryShape === 'square') {
                    const normT = (t / (Math.PI * 2)) * 4;
                    const phase = Math.floor(normT);
                    const subT = normT - phase;
                    const half = r * 0.7;
                    if (phase === 0) { tx = 4 - half + subT * 2 * half; tz = 3 - half; ty = 4; }
                    else if (phase === 1) { tx = 4 + half; tz = 3 - half + subT * 2 * half; ty = 5; }
                    else if (phase === 2) { tx = 4 + half - subT * 2 * half; tz = 3 + half; ty = 4; }
                    else { tx = 4 - half; tz = 3 + half - subT * 2 * half; ty = 3; }
                } else {
                    tx = 2 + (t / (Math.PI * 2)) * (maxReach * 0.5);
                    ty = 4 + (r * 0.6) * Math.sin(t * 3);
                    tz = 3 + (r * 0.4) * Math.cos(t * 2);
                }

                visualizer3D.target.set(tx, ty, tz);
                visualizer3D.targetMesh.position.set(tx, ty, tz);
                visualizer3D.solve3DIK(tx, ty, tz);

                document.getElementById('inputTargetX').value = tx.toFixed(2);
                document.getElementById('inputTargetY').value = ty.toFixed(2);
                document.getElementById('inputTargetZ').value = tz.toFixed(2);
            } else {
                if (state.trajectoryShape === 'circle') {
                    tx = cx + r * Math.cos(t);
                    ty = cy + r * Math.sin(t);
                } else if (state.trajectoryShape === 'lemniscate') {
                    tx = cx + r * Math.sin(t);
                    ty = cy + (r * 0.6) * Math.sin(2 * t);
                } else {
                    tx = (maxReach * 0.2) + (t / (Math.PI * 2)) * (maxReach * 0.55);
                    ty = cy + (r * 0.7) * Math.sin(t * 3);
                }
                visualizer2D.target.x = tx;
                visualizer2D.target.y = ty;
                robot.solveCCD(tx, ty, 15, 0.01);
                visualizer2D.render();
            }

            updateJointSliders();
            updateTelemetry();

            state.trajectoryTimer = requestAnimationFrame(animate);
        }

        state.trajectoryTimer = requestAnimationFrame(animate);
    }

    function stopTrajectory() {
        state.isPlayingTrajectory = false;
        if (state.trajectoryTimer) {
            cancelAnimationFrame(state.trajectoryTimer);
            state.trajectoryTimer = null;
        }
        const btnPlay = document.getElementById('btnPlayTrajectory');
        if (btnPlay) {
            btnPlay.innerHTML = `
                <svg width="14" height="14" viewBox="0 0 24 24" fill="currentColor"><polygon points="5 3 19 12 5 21 5 3"/></svg>
                Play Trajectory
            `;
            btnPlay.classList.remove('playing');
        }
    }

    /**
     * Generate Python code supporting 3D and 2D visualization
     */
    function generatePythonCode() {
        const pyTextarea = document.getElementById('pythonCodeOutput');
        if (!pyTextarea) return;

        const dof = robot.dof;
        const angles = robot.angles;
        const links = robot.links;

        const code = `#!/usr/bin/env python3
"""
${dof}-DOF Robot Arm Kinematics (3D & 2D Matplotlib)
Forward and Inverse Kinematics generated from RoboKinematics.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Current Arm Configuration
LINK_LENGTHS = [${links.map(l => l.toFixed(1)).join(', ')}]
JOINT_ANGLES_DEG = [${angles.map(a => a.toFixed(1)).join(', ')}]

def forward_kinematics_3d(angles_deg, lengths, base_height=1.5):
    """
    Computes 3D joint positions:
    - angles_deg[0]: Base Yaw (azimuth around vertical Z-axis)
    - angles_deg[1:]: Elevation pitch angles
    """
    angles_rad = np.radians(angles_deg)
    yaw = angles_rad[0]
    pitches = angles_rad[1:]

    pts = [np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, base_height])]
    cum_pitch = np.cumsum(pitches)

    r = 0.0
    z = base_height
    for i in range(len(pitches)):
        r += lengths[i+1] * np.cos(cum_pitch[i])
        z += lengths[i+1] * np.sin(cum_pitch[i])
        x = r * np.cos(yaw)
        y = r * np.sin(yaw)
        pts.append(np.array([x, y, z]))

    return pts

def solve_inverse_kinematics_3d(target_x, target_y, target_z, lengths, base_height=1.5, max_iter=50):
    """
    Solves 3D IK using Base Azimuth + Planar CCD.
    """
    # 1. Base Yaw
    yaw = np.arctan2(target_y, target_x)

    # 2. Planar Elevation Target
    planar_r = np.hypot(target_x, target_y)
    planar_z = target_z - base_height

    planar_lengths = lengths[1:]
    pitches = np.zeros(len(planar_lengths))

    for _ in range(max_iter):
        for i in reversed(range(len(planar_lengths))):
            cum = np.cumsum(pitches)
            ee_x = np.sum(planar_lengths * np.cos(cum))
            ee_z = np.sum(planar_lengths * np.sin(cum))

            err = np.hypot(planar_r - ee_x, planar_z - ee_z)
            if err < 1e-3:
                break

            jx = 0.0 if i == 0 else np.sum(planar_lengths[:i] * np.cos(cum[:i]))
            jz = 0.0 if i == 0 else np.sum(planar_lengths[:i] * np.sin(cum[:i]))

            v_ee = np.array([ee_x - jx, ee_z - jz])
            v_tgt = np.array([planar_r - jx, planar_z - jz])

            norm_ee = np.linalg.norm(v_ee)
            norm_tgt = np.linalg.norm(v_tgt)
            if norm_ee > 1e-6 and norm_tgt > 1e-6:
                cos_a = np.clip(np.dot(v_ee, v_tgt) / (norm_ee * norm_tgt), -1.0, 1.0)
                sin_a = (v_ee[0] * v_tgt[1] - v_ee[1] * v_tgt[0]) / (norm_ee * norm_tgt)
                pitches[i] += np.arctan2(sin_a, cos_a) * 0.8

    solved_deg = [np.degrees(yaw)] + list(np.degrees(pitches))
    return solved_deg

if __name__ == "__main__":
    print("Visualizing 3D Robot Arm with Matplotlib...")
    pts = forward_kinematics_3d(JOINT_ANGLES_DEG, LINK_LENGTHS)
    xs, ys, zs = [p[0] for p in pts], [p[1] for p in pts], [p[2] for p in pts]

    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("${dof}-DOF Robot Arm (3D Kinematics)", fontsize=14, pad=15)

    colors = ['k-', 'r-', 'g-', 'b-', 'm-', 'y-']
    for i in range(len(pts) - 1):
        ax.plot([xs[i], xs[i+1]], [ys[i], ys[i+1]], [zs[i], zs[i+1]], colors[i % len(colors)], linewidth=4)
        ax.scatter(xs[i], ys[i], zs[i], color='black', s=50)

    ax.scatter(xs[-1], ys[-1], zs[-1], color='red', s=120, label="End-Effector")
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z (Height)')
    ax.legend()
    plt.show()
`;
        pyTextarea.value = code;
    }

    function showToast(msg) {
        let toast = document.getElementById('toast');
        if (!toast) {
            toast = document.createElement('div');
            toast.id = 'toast';
            toast.className = 'toast-notification';
            document.body.appendChild(toast);
        }
        toast.textContent = msg;
        toast.classList.add('show');
        setTimeout(() => toast.classList.remove('show'), 2500);
    }
});
