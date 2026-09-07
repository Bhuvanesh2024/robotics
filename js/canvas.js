/**
 * Interactive Canvas Visualizer for Robot Arm
 * Supports Pan, Zoom, High-DPI, Drag-and-Drop IK Target,
 * Matplotlib-style coordinate tags, angle arcs, workspace envelope, and trail.
 */

class RobotCanvas {
    constructor(canvasElement, robot) {
        this.canvas = canvasElement;
        this.ctx = canvasElement.getContext('2d');
        this.robot = robot;

        // Viewport transform
        this.scale = 35; // Pixels per world unit (will auto-fit)
        this.pan = { x: 0, y: 0 };
        this.isPanning = false;
        this.panStart = { x: 0, y: 0 };

        // Interaction state
        this.target = { x: 8, y: 4 };
        this.isDraggingTarget = false;
        this.isDraggingJoint = -1; // Joint index if dragging an angle directly
        this.hoveredJoint = -1;
        this.hoveredTarget = false;

        // Display options
        this.showGrid = true;
        this.showAxes = true;
        this.showLabels = true;
        this.showWorkspace = true;
        this.showAngleArcs = true;
        this.showTrail = true;
        this.trailPoints = [];
        this.maxTrailLength = 400;

        // Link colors (matching user's matplotlib colors: red, green, blue, magenta)
        this.linkColors = [
            '#ef4444', // Link 1 - Red
            '#10b981', // Link 2 - Green
            '#3b82f6', // Link 3 - Blue
            '#d946ef', // Link 4 - Magenta
            '#f59e0b', // Link 5 - Amber
            '#06b6d4'  // Link 6 - Cyan
        ];

        this.targetColor = '#f97316';

        // Callbacks
        this.onTargetChange = null;
        this.onJointAngleChange = null;

        this.initEventListeners();
        this.resize();
    }

    resize() {
        const rect = this.canvas.getBoundingClientRect();
        const dpr = window.devicePixelRatio || 1;
        this.canvas.width = rect.width * dpr;
        this.canvas.height = rect.height * dpr;
        this.ctx.resetTransform();
        this.ctx.scale(dpr, dpr);
        this.width = rect.width;
        this.height = rect.height;

        // Auto center pan if uninitialized
        if (this.pan.x === 0 && this.pan.y === 0) {
            this.pan.x = this.width / 2;
            this.pan.y = this.height / 2;
        }
        this.render();
    }

    fitToScreen() {
        const fk = this.robot.forwardKinematics();
        const maxR = fk.maxReach + 2;
        const minDim = Math.min(this.width, this.height);
        this.scale = (minDim / 2) / maxR;
        this.pan.x = this.width / 2;
        this.pan.y = this.height / 2;
        this.render();
    }

    worldToScreen(wx, wy) {
        return {
            x: this.pan.x + wx * this.scale,
            y: this.pan.y - wy * this.scale // Invert Y for standard Cartesian axes
        };
    }

    screenToWorld(sx, sy) {
        return {
            x: (sx - this.pan.x) / this.scale,
            y: (this.pan.y - sy) / this.scale
        };
    }

    initEventListeners() {
        if (typeof ResizeObserver !== 'undefined') {
            new ResizeObserver(() => this.resize()).observe(this.canvas);
        }
        window.addEventListener('resize', () => this.resize());

        // Mouse Down
        this.canvas.addEventListener('mousedown', (e) => {
            const rect = this.canvas.getBoundingClientRect();
            const sx = e.clientX - rect.left;
            const sy = e.clientY - rect.top;
            const world = this.screenToWorld(sx, sy);

            if (e.button === 1 || e.button === 2 || e.shiftKey || e.altKey) {
                // Middle / Right click or Shift+drag: Pan
                this.isPanning = true;
                this.panStart = { x: sx - this.pan.x, y: sy - this.pan.y };
                e.preventDefault();
                return;
            }

            // Check if clicking near target
            const targetScreen = this.worldToScreen(this.target.x, this.target.y);
            const distToTarget = Math.hypot(sx - targetScreen.x, sy - targetScreen.y);

            if (distToTarget < 22) {
                this.isDraggingTarget = true;
                this.canvas.style.cursor = 'grabbing';
                return;
            }

            // Check if clicking near any joint pivot
            const fk = this.robot.forwardKinematics();
            for (let i = fk.positions.length - 1; i >= 0; i--) {
                const jScreen = this.worldToScreen(fk.positions[i].x, fk.positions[i].y);
                const d = Math.hypot(sx - jScreen.x, sy - jScreen.y);
                if (d < 16) {
                    if (i === fk.positions.length - 1) {
                        // Clicking end-effector behaves like grabbing target
                        this.target.x = fk.positions[i].x;
                        this.target.y = fk.positions[i].y;
                        this.isDraggingTarget = true;
                        this.canvas.style.cursor = 'grabbing';
                    } else if (i > 0) {
                        this.isDraggingJoint = i;
                        this.canvas.style.cursor = 'grabbing';
                    }
                    return;
                }
            }

            // If left-click on empty space, move target there and begin drag
            this.target.x = world.x;
            this.target.y = world.y;
            this.isDraggingTarget = true;
            if (this.onTargetChange) this.onTargetChange(this.target.x, this.target.y);
            this.render();
        });

        // Mouse Move
        window.addEventListener('mousemove', (e) => {
            const rect = this.canvas.getBoundingClientRect();
            const sx = e.clientX - rect.left;
            const sy = e.clientY - rect.top;

            if (this.isPanning) {
                this.pan.x = sx - this.panStart.x;
                this.pan.y = sy - this.panStart.y;
                this.render();
                return;
            }

            const world = this.screenToWorld(sx, sy);

            if (this.isDraggingTarget) {
                this.target.x = world.x;
                this.target.y = world.y;
                if (this.onTargetChange) this.onTargetChange(this.target.x, this.target.y);
                this.render();
                return;
            }

            if (this.isDraggingJoint > 0) {
                // Dragging a joint pivot to manually adjust angle
                const fk = this.robot.forwardKinematics();
                const prevJoint = fk.positions[this.isDraggingJoint - 1];
                const dx = world.x - prevJoint.x;
                const dy = world.y - prevJoint.y;
                const globalAngle = Math.atan2(dy, dx);
                
                // Absolute angle of link before
                let prevGlobalAngle = 0;
                for (let k = 0; k < this.isDraggingJoint - 1; k++) {
                    prevGlobalAngle += (this.robot.angles[k] * Math.PI) / 180;
                }

                let relRad = globalAngle - prevGlobalAngle;
                while (relRad > Math.PI) relRad -= 2 * Math.PI;
                while (relRad < -Math.PI) relRad += 2 * Math.PI;

                const deg = (relRad * 180) / Math.PI;
                if (this.onJointAngleChange) {
                    this.onJointAngleChange(this.isDraggingJoint - 1, deg);
                }
                this.render();
                return;
            }

            // Hover detection
            const targetScreen = this.worldToScreen(this.target.x, this.target.y);
            const distToTarget = Math.hypot(sx - targetScreen.x, sy - targetScreen.y);
            this.hoveredTarget = distToTarget < 20;

            const fk = this.robot.forwardKinematics();
            let hoveredJ = -1;
            for (let i = 0; i < fk.positions.length; i++) {
                const jScreen = this.worldToScreen(fk.positions[i].x, fk.positions[i].y);
                if (Math.hypot(sx - jScreen.x, sy - jScreen.y) < 14) {
                    hoveredJ = i;
                    break;
                }
            }
            this.hoveredJoint = hoveredJ;

            if (this.hoveredTarget || this.hoveredJoint >= 0) {
                this.canvas.style.cursor = 'grab';
            } else {
                this.canvas.style.cursor = 'crosshair';
            }
        });

        // Mouse Up
        window.addEventListener('mouseup', () => {
            this.isPanning = false;
            this.isDraggingTarget = false;
            this.isDraggingJoint = -1;
            this.canvas.style.cursor = 'crosshair';
        });

        // Context Menu prevent
        this.canvas.addEventListener('contextmenu', (e) => e.preventDefault());

        // Mouse Wheel Zoom
        this.canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            const rect = this.canvas.getBoundingClientRect();
            const sx = e.clientX - rect.left;
            const sy = e.clientY - rect.top;

            const worldBefore = this.screenToWorld(sx, sy);
            const zoomFactor = e.deltaY < 0 ? 1.12 : 0.89;
            const newScale = Math.max(5, Math.min(300, this.scale * zoomFactor));

            // Zoom centered on cursor
            this.scale = newScale;
            this.pan.x = sx - worldBefore.x * this.scale;
            this.pan.y = sy + worldBefore.y * this.scale;

            this.render();
        }, { passive: false });

        // Touch support for tablets/smartphones
        let touchStartDist = 0;
        this.canvas.addEventListener('touchstart', (e) => {
            if (e.touches.length === 1) {
                const rect = this.canvas.getBoundingClientRect();
                const sx = e.touches[0].clientX - rect.left;
                const sy = e.touches[0].clientY - rect.top;
                const world = this.screenToWorld(sx, sy);
                this.target.x = world.x;
                this.target.y = world.y;
                this.isDraggingTarget = true;
                if (this.onTargetChange) this.onTargetChange(this.target.x, this.target.y);
                this.render();
            } else if (e.touches.length === 2) {
                touchStartDist = Math.hypot(
                    e.touches[0].clientX - e.touches[1].clientX,
                    e.touches[0].clientY - e.touches[1].clientY
                );
            }
        }, { passive: false });

        this.canvas.addEventListener('touchmove', (e) => {
            e.preventDefault();
            if (e.touches.length === 1 && this.isDraggingTarget) {
                const rect = this.canvas.getBoundingClientRect();
                const sx = e.touches[0].clientX - rect.left;
                const sy = e.touches[0].clientY - rect.top;
                const world = this.screenToWorld(sx, sy);
                this.target.x = world.x;
                this.target.y = world.y;
                if (this.onTargetChange) this.onTargetChange(this.target.x, this.target.y);
                this.render();
            } else if (e.touches.length === 2) {
                const dist = Math.hypot(
                    e.touches[0].clientX - e.touches[1].clientX,
                    e.touches[0].clientY - e.touches[1].clientY
                );
                if (touchStartDist > 0) {
                    const factor = dist / touchStartDist;
                    this.scale = Math.max(5, Math.min(300, this.scale * factor));
                    touchStartDist = dist;
                    this.render();
                }
            }
        }, { passive: false });

        this.canvas.addEventListener('touchend', () => {
            this.isDraggingTarget = false;
            touchStartDist = 0;
        });
    }

    addTrailPoint(x, y) {
        this.trailPoints.push({ x, y });
        if (this.trailPoints.length > this.maxTrailLength) {
            this.trailPoints.shift();
        }
    }

    clearTrail() {
        this.trailPoints = [];
        this.render();
    }

    // Main Draw Function
    render() {
        const ctx = this.ctx;
        const w = this.width;
        const h = this.height;

        // Clear canvas
        ctx.clearRect(0, 0, w, h);

        // Draw grid and axes
        if (this.showGrid) this.drawGrid();
        if (this.showWorkspace) this.drawWorkspace();
        if (this.showAxes) this.drawAxes();
        if (this.showTrail) this.drawTrail();

        // Compute Forward Kinematics
        const fk = this.robot.forwardKinematics();
        const pts = fk.positions;
        const n = this.robot.dof;

        // Add trail point of End-Effector
        const ee = pts[pts.length - 1];
        this.addTrailPoint(ee.x, ee.y);

        // Draw Links
        this.drawRobotLinks(pts);

        // Draw Joint angle arcs
        if (this.showAngleArcs) {
            this.drawAngleArcs(pts, fk.cumulativeAnglesRad);
        }

        // Draw Joint pivots & bearing graphics
        this.drawJointPivots(pts);

        // Draw Coordinate Labels matching matplotlib ax.text
        if (this.showLabels) {
            this.drawCoordinateLabels(pts);
        }

        // Draw Target crosshair
        this.drawTargetMarker();
    }

    drawGrid() {
        const ctx = this.ctx;
        const leftWorld = this.screenToWorld(0, this.height);
        const rightWorld = this.screenToWorld(this.width, 0);

        // Grid unit calculation based on zoom level
        let step = 1;
        if (this.scale < 15) step = 5;
        if (this.scale < 7) step = 10;
        if (this.scale > 80) step = 0.5;
        if (this.scale > 160) step = 0.2;

        const startX = Math.floor(leftWorld.x / step) * step;
        const endX = Math.ceil(rightWorld.x / step) * step;
        const startY = Math.floor(leftWorld.y / step) * step;
        const endY = Math.ceil(rightWorld.y / step) * step;

        ctx.lineWidth = 1;

        for (let x = startX; x <= endX; x += step) {
            const p = this.worldToScreen(x, 0);
            ctx.strokeStyle = Math.abs(x) < 1e-4 ? 'rgba(148, 163, 184, 0.4)' : 'rgba(148, 163, 184, 0.12)';
            ctx.beginPath();
            ctx.moveTo(p.x, 0);
            ctx.lineTo(p.x, this.height);
            ctx.stroke();

            // Coordinate number on axis
            if (Math.abs(x) > 1e-4 && Math.abs(x % (step * 2)) < 1e-4) {
                const originScreen = this.worldToScreen(0, 0);
                ctx.fillStyle = 'rgba(148, 163, 184, 0.6)';
                ctx.font = '10px ui-monospace, SFMono-Regular, monospace';
                ctx.fillText(x.toFixed(step < 1 ? 1 : 0), p.x + 3, Math.min(this.height - 10, Math.max(15, originScreen.y + 12)));
            }
        }

        for (let y = startY; y <= endY; y += step) {
            const p = this.worldToScreen(0, y);
            ctx.strokeStyle = Math.abs(y) < 1e-4 ? 'rgba(148, 163, 184, 0.4)' : 'rgba(148, 163, 184, 0.12)';
            ctx.beginPath();
            ctx.moveTo(0, p.y);
            ctx.lineTo(this.width, p.y);
            ctx.stroke();

            if (Math.abs(y) > 1e-4 && Math.abs(y % (step * 2)) < 1e-4) {
                const originScreen = this.worldToScreen(0, 0);
                ctx.fillStyle = 'rgba(148, 163, 184, 0.6)';
                ctx.font = '10px ui-monospace, SFMono-Regular, monospace';
                ctx.fillText(y.toFixed(step < 1 ? 1 : 0), Math.min(this.width - 25, Math.max(10, originScreen.x + 5)), p.y - 3);
            }
        }
    }

    drawAxes() {
        const ctx = this.ctx;
        const origin = this.worldToScreen(0, 0);

        // X Axis
        ctx.strokeStyle = 'rgba(239, 68, 68, 0.6)'; // Reddish for X
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(0, origin.y);
        ctx.lineTo(this.width, origin.y);
        ctx.stroke();

        // Y Axis
        ctx.strokeStyle = 'rgba(34, 197, 94, 0.6)'; // Greenish for Y
        ctx.beginPath();
        ctx.moveTo(origin.x, 0);
        ctx.lineTo(origin.x, this.height);
        ctx.stroke();

        // Origin label
        ctx.fillStyle = 'rgba(255, 255, 255, 0.7)';
        ctx.font = '11px sans-serif';
        ctx.fillText('(0,0)', origin.x - 28, origin.y + 16);
    }

    drawWorkspace() {
        const ctx = this.ctx;
        const fk = this.robot.forwardKinematics();
        const origin = this.worldToScreen(0, 0);

        // Max Reach Circle
        const maxRScreen = fk.maxReach * this.scale;
        ctx.beginPath();
        ctx.arc(origin.x, origin.y, maxRScreen, 0, Math.PI * 2);
        ctx.strokeStyle = 'rgba(56, 189, 248, 0.25)';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([6, 6]);
        ctx.stroke();

        // Reachable area translucent fill
        ctx.fillStyle = 'rgba(56, 189, 248, 0.03)';
        ctx.fill();

        // Min Reach Circle (blind spot / void at base if links can't fold)
        if (fk.minReach > 0.05) {
            const minRScreen = fk.minReach * this.scale;
            ctx.beginPath();
            ctx.arc(origin.x, origin.y, minRScreen, 0, Math.PI * 2);
            ctx.strokeStyle = 'rgba(244, 63, 94, 0.25)';
            ctx.stroke();
        }

        ctx.setLineDash([]);
    }

    drawTrail() {
        if (this.trailPoints.length < 2) return;
        const ctx = this.ctx;
        ctx.lineWidth = 2.5;

        for (let i = 1; i < this.trailPoints.length; i++) {
            const p1 = this.worldToScreen(this.trailPoints[i - 1].x, this.trailPoints[i - 1].y);
            const p2 = this.worldToScreen(this.trailPoints[i].x, this.trailPoints[i].y);
            const alpha = (i / this.trailPoints.length) * 0.7;

            ctx.beginPath();
            ctx.moveTo(p1.x, p1.y);
            ctx.lineTo(p2.x, p2.y);
            ctx.strokeStyle = `rgba(217, 70, 239, ${alpha})`;
            ctx.stroke();
        }
    }

    drawRobotLinks(pts) {
        const ctx = this.ctx;

        // Base pedestal stand
        const bScreen = this.worldToScreen(pts[0].x, pts[0].y);
        ctx.fillStyle = '#334155';
        ctx.beginPath();
        ctx.ellipse(bScreen.x, bScreen.y + 6, 20, 8, 0, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = '#64748b';
        ctx.lineWidth = 2;
        ctx.stroke();

        // Draw each link segment with rounded casing
        for (let i = 0; i < pts.length - 1; i++) {
            const p1 = this.worldToScreen(pts[i].x, pts[i].y);
            const p2 = this.worldToScreen(pts[i + 1].x, pts[i + 1].y);
            const color = this.linkColors[i % this.linkColors.length];

            // Outer glow / shadow
            ctx.shadowColor = color;
            ctx.shadowBlur = 10;

            // Main Link Line (matching matplotlib linewidth 3 or modern 6px)
            ctx.beginPath();
            ctx.moveTo(p1.x, p1.y);
            ctx.lineTo(p2.x, p2.y);
            ctx.strokeStyle = color;
            ctx.lineWidth = 6;
            ctx.lineCap = 'round';
            ctx.stroke();

            // Inner highlight core
            ctx.shadowBlur = 0;
            ctx.beginPath();
            ctx.moveTo(p1.x, p1.y);
            ctx.lineTo(p2.x, p2.y);
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.4)';
            ctx.lineWidth = 2;
            ctx.stroke();

            // Link number badge midway
            const midX = (p1.x + p2.x) / 2;
            const midY = (p1.y + p2.y) / 2;
            const angle = Math.atan2(p2.y - p1.y, p2.x - p1.x);
            const perpX = -Math.sin(angle) * 14;
            const perpY = Math.cos(angle) * 14;

            ctx.fillStyle = 'rgba(15, 23, 42, 0.85)';
            ctx.beginPath();
            ctx.arc(midX + perpX, midY + perpY, 9, 0, Math.PI * 2);
            ctx.fill();
            ctx.strokeStyle = color;
            ctx.lineWidth = 1.5;
            ctx.stroke();

            ctx.fillStyle = '#f8fafc';
            ctx.font = 'bold 9px sans-serif';
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            ctx.fillText(`L${i + 1}`, midX + perpX, midY + perpY);
        }
    }

    drawAngleArcs(pts, cumulativeAnglesRad) {
        const ctx = this.ctx;
        const arcRadius = 26;

        for (let i = 0; i < pts.length - 1; i++) {
            const p = this.worldToScreen(pts[i].x, pts[i].y);
            const prevAbsRad = i === 0 ? 0 : cumulativeAnglesRad[i - 1];
            const currAbsRad = cumulativeAnglesRad[i];

            // Canvas y is inverted: canvas angle = -world angle
            const startCanvasAngle = -prevAbsRad;
            const endCanvasAngle = -currAbsRad;
            const isCounterClockwise = endCanvasAngle < startCanvasAngle;

            ctx.beginPath();
            ctx.arc(p.x, p.y, arcRadius, startCanvasAngle, endCanvasAngle, isCounterClockwise);
            ctx.strokeStyle = 'rgba(251, 191, 36, 0.8)'; // Golden arc
            ctx.lineWidth = 1.5;
            ctx.stroke();

            // Angle arc fill sector
            ctx.fillStyle = 'rgba(251, 191, 36, 0.12)';
            ctx.beginPath();
            ctx.moveTo(p.x, p.y);
            ctx.arc(p.x, p.y, arcRadius, startCanvasAngle, endCanvasAngle, isCounterClockwise);
            ctx.closePath();
            ctx.fill();

            // Small theta label
            const midAngle = (startCanvasAngle + endCanvasAngle) / 2;
            const tx = p.x + Math.cos(midAngle) * (arcRadius + 10);
            const ty = p.y + Math.sin(midAngle) * (arcRadius + 10);
            ctx.fillStyle = '#fbbf24';
            ctx.font = '10px sans-serif';
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            ctx.fillText(`θ${i + 1}`, tx, ty);
        }
    }

    drawJointPivots(pts) {
        const ctx = this.ctx;

        for (let i = 0; i < pts.length; i++) {
            const p = this.worldToScreen(pts[i].x, pts[i].y);
            const isEndEffector = (i === pts.length - 1);
            const isHovered = (this.hoveredJoint === i);

            if (isEndEffector) {
                // End-effector gripper / tool head
                ctx.shadowColor = '#ef4444';
                ctx.shadowBlur = 12;

                ctx.beginPath();
                ctx.arc(p.x, p.y, isHovered ? 11 : 9, 0, Math.PI * 2);
                ctx.fillStyle = '#ef4444'; // Red matching ax.plot(x, y, 'ro')
                ctx.fill();
                ctx.strokeStyle = '#ffffff';
                ctx.lineWidth = 2.5;
                ctx.stroke();

                // Inner pin dot
                ctx.beginPath();
                ctx.arc(p.x, p.y, 3, 0, Math.PI * 2);
                ctx.fillStyle = '#ffffff';
                ctx.fill();

                ctx.shadowBlur = 0;
            } else {
                // Intermediate joints (matching user's ax.plot(..., 'ko', markersize=8))
                ctx.beginPath();
                ctx.arc(p.x, p.y, isHovered ? 9 : 7, 0, Math.PI * 2);
                ctx.fillStyle = '#0f172a'; // Dark core
                ctx.fill();
                ctx.strokeStyle = '#cbd5e1'; // Light metal outer bearing
                ctx.lineWidth = 2.5;
                ctx.stroke();

                // Small center pivot pin
                ctx.beginPath();
                ctx.arc(p.x, p.y, 2.5, 0, Math.PI * 2);
                ctx.fillStyle = this.linkColors[i % this.linkColors.length];
                ctx.fill();
            }
        }
    }

    drawCoordinateLabels(pts) {
        const ctx = this.ctx;

        ctx.font = '11px ui-monospace, SFMono-Regular, monospace';
        ctx.textAlign = 'left';
        ctx.textBaseline = 'bottom';

        for (let i = 0; i < pts.length; i++) {
            const p = this.worldToScreen(pts[i].x, pts[i].y);
            const isEndEffector = (i === pts.length - 1);

            const text = `(${pts[i].x.toFixed(2)}, ${pts[i].y.toFixed(2)})`;

            // Text background pill for perfect readability
            const metrics = ctx.measureText(text);
            const textW = metrics.width;
            const pad = 4;
            const boxX = p.x + 8;
            const boxY = p.y - 18;

            ctx.fillStyle = 'rgba(15, 23, 42, 0.88)';
            ctx.beginPath();
            ctx.roundRect(boxX, boxY, textW + pad * 2, 18, 4);
            ctx.fill();
            ctx.strokeStyle = isEndEffector ? '#ef4444' : 'rgba(148, 163, 184, 0.4)';
            ctx.lineWidth = 1;
            ctx.stroke();

            // Matplotlib text color: red for end effector, white for others
            ctx.fillStyle = isEndEffector ? '#f87171' : '#e2e8f0';
            ctx.fillText(text, boxX + pad, boxY + 13);
        }
    }

    drawTargetMarker() {
        const ctx = this.ctx;
        const p = this.worldToScreen(this.target.x, this.target.y);

        // Pulsing / interactive target crosshair
        const r = this.isDraggingTarget ? 15 : 12;

        ctx.shadowColor = this.targetColor;
        ctx.shadowBlur = this.isDraggingTarget ? 15 : 8;

        // Outer dashed ring
        ctx.beginPath();
        ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
        ctx.strokeStyle = this.targetColor;
        ctx.lineWidth = 2;
        ctx.stroke();

        // Crosshairs
        ctx.beginPath();
        ctx.moveTo(p.x - r - 4, p.y);
        ctx.lineTo(p.x + r + 4, p.y);
        ctx.moveTo(p.x, p.y - r - 4);
        ctx.lineTo(p.x + r + 4, p.y);
        ctx.stroke();

        // Center dot
        ctx.beginPath();
        ctx.arc(p.x, p.y, 3.5, 0, Math.PI * 2);
        ctx.fillStyle = this.targetColor;
        ctx.fill();

        ctx.shadowBlur = 0;

        // Label above target
        ctx.fillStyle = this.targetColor;
        ctx.font = '10px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText(`Target (${this.target.x.toFixed(2)}, ${this.target.y.toFixed(2)})`, p.x, p.y - r - 8);
    }

    setTheme(theme) {
        if (theme === 'cyber') {
            this.targetColor = '#38bdf8';
        } else if (theme === 'emerald') {
            this.targetColor = '#10b981';
        } else if (theme === 'purple') {
            this.targetColor = '#c084fc';
        } else if (theme === 'gold') {
            this.targetColor = '#eab308';
        } else if (theme === 'light') {
            this.targetColor = '#0284c7';
        } else {
            this.targetColor = '#f97316'; // Industrial Orange
        }
        this.render();
    }
}

// Export for browser
if (typeof window !== 'undefined') {
    window.RobotCanvas = RobotCanvas;
}
