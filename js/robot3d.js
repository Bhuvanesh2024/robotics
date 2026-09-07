/**
 * 3D Robot Arm Kinematics & Three.js Visualizer
 * Supports:
 *  - 3D Forward Kinematics (FK)
 *  - 3D Inverse Kinematics (IK) with Base Yaw + Planar Elevation / 3D CCD
 *  - High-fidelity articulated 3D robot model with PBR materials, shadows, and servo joints
 *  - 3D End-Effector Gripper (open/close animation)
 *  - 3D TransformControls (interactive drag gizmo)
 *  - 3D Reachable Workspace sphere & 3D motion trails
 *  - Camera presets (Isometric, Front, Top, Side, Close-up)
 */

class Robot3DVisualizer {
    constructor(containerElement, robotKinematics) {
        this.container = containerElement;
        this.robot = robotKinematics;

        // Base height offset
        this.baseHeight = 1.5;

        // Link colors matching 2D
        this.colors = [
            0xef4444, // Link 1 - Red
            0x10b981, // Link 2 - Green
            0x3b82f6, // Link 3 - Blue
            0xd946ef, // Link 4 - Magenta
            0xf59e0b, // Link 5 - Amber
            0x06b6d4  // Link 6 - Cyan
        ];

        // 3D Target
        this.target = new THREE.Vector3(6, 4, 3);
        this.gripperOpen = 0.8; // 0 (closed) to 1 (wide open)

        // Display options
        this.showWorkspace = true;
        this.showTrail = true;
        this.showGizmo = true;
        this.trailPoints = [];
        this.maxTrailPoints = 350;

        // Callbacks
        this.onTargetChange = null;
        this.onAnglesChange = null;

        this.initThree();
        this.buildRobot();
        this.buildEnvironment();
        this.buildWorkspaceSphere();
        this.buildTrailRenderer();
        this.setupTransformControls();
        this.setupEventListeners();
        this.updateRobotPose();
    }

    initThree() {
        const width = this.container.clientWidth || 800;
        const height = this.container.clientHeight || 600;

        // Scene
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0a0f1d);
        this.scene.fog = new THREE.FogExp2(0x0a0f1d, 0.015);

        // Camera
        this.camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 1000);
        this.camera.position.set(16, 14, 20);

        // Renderer with Antialiasing and Soft Shadows
        this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: 'high-performance' });
        this.renderer.setSize(width, height);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
        this.renderer.toneMappingExposure = 1.1;
        this.container.appendChild(this.renderer.domElement);

        // OrbitControls
        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        this.controls.target.set(0, 4, 0);
        this.controls.maxPolarAngle = Math.PI / 2 + 0.05; // Don't go deep under floor
        this.controls.minDistance = 3;
        this.controls.maxDistance = 60;

        // Lighting
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.55);
        this.scene.add(ambientLight);

        // Main Key Spotlight
        this.dirLight = new THREE.DirectionalLight(0xffffff, 1.2);
        this.dirLight.position.set(20, 30, 20);
        this.dirLight.castShadow = true;
        this.dirLight.shadow.mapSize.width = 2048;
        this.dirLight.shadow.mapSize.height = 2048;
        this.dirLight.shadow.camera.near = 0.5;
        this.dirLight.shadow.camera.far = 80;
        const d = 25;
        this.dirLight.shadow.camera.left = -d;
        this.dirLight.shadow.camera.right = d;
        this.dirLight.shadow.camera.top = d;
        this.dirLight.shadow.camera.bottom = -d;
        this.dirLight.shadow.bias = -0.0005;
        this.scene.add(this.dirLight);

        // Blue Rim Light
        const rimLight = new THREE.DirectionalLight(0x38bdf8, 0.8);
        rimLight.position.set(-20, 15, -15);
        this.scene.add(rimLight);

        // Purple Fill Light
        const fillLight = new THREE.PointLight(0xd946ef, 0.6, 50);
        fillLight.position.set(0, 2, 10);
        this.scene.add(fillLight);
    }

    buildEnvironment() {
        // High-tech Cyber/Robotics Grid Floor
        const floorGeo = new THREE.PlaneGeometry(80, 80);
        const floorMat = new THREE.MeshStandardMaterial({
            color: 0x0f172a,
            roughness: 0.85,
            metalness: 0.2
        });
        const floor = new THREE.Mesh(floorGeo, floorMat);
        floor.rotation.x = -Math.PI / 2;
        floor.receiveShadow = true;
        this.floor = floor;
        this.scene.add(floor);

        // Grid lines
        this.gridHelper = new THREE.GridHelper(80, 80, 0xf97316, 0x1f2733);
        this.gridHelper.position.y = 0.01;
        this.scene.add(this.gridHelper);

        // Origin axes helper
        const axesHelper = new THREE.AxesHelper(3);
        axesHelper.position.set(0, 0.02, 0);
        this.scene.add(axesHelper);

        // Pedestal Platform for Robot
        const pedestalGeo = new THREE.CylinderGeometry(2.5, 3.2, 0.4, 32);
        const pedestalMat = new THREE.MeshStandardMaterial({
            color: 0x1e293b,
            metalness: 0.7,
            roughness: 0.3
        });
        const pedestal = new THREE.Mesh(pedestalGeo, pedestalMat);
        pedestal.position.y = 0.2;
        pedestal.receiveShadow = true;
        pedestal.castShadow = true;
        this.scene.add(pedestal);

        // Outer glowing ring around base
        const ringGeo = new THREE.RingGeometry(3.2, 3.4, 64);
        const ringMat = new THREE.MeshBasicMaterial({ color: 0xf97316, side: THREE.DoubleSide });
        this.pedestalRing = new THREE.Mesh(ringGeo, ringMat);
        this.pedestalRing.rotation.x = -Math.PI / 2;
        this.pedestalRing.position.y = 0.02;
        this.scene.add(this.pedestalRing);
    }

    setTheme(theme) {
        let bgColor = 0x0b0f14;
        let gridMajor = 0xf97316;
        let gridMinor = 0x1f2733;
        let floorColor = 0x141a22;
        let accentColor = 0xf97316;

        if (theme === 'cyber') {
            bgColor = 0x0a0f1d;
            gridMajor = 0x38bdf8;
            gridMinor = 0x1e293b;
            floorColor = 0x0f172a;
            accentColor = 0x38bdf8;
        } else if (theme === 'emerald') {
            bgColor = 0x06110f;
            gridMajor = 0x10b981;
            gridMinor = 0x132723;
            floorColor = 0x0b1a17;
            accentColor = 0x10b981;
        } else if (theme === 'purple') {
            bgColor = 0x0d091a;
            gridMajor = 0xc084fc;
            gridMinor = 0x231b40;
            floorColor = 0x16102b;
            accentColor = 0xc084fc;
        } else if (theme === 'gold') {
            bgColor = 0x0f1012;
            gridMajor = 0xeab308;
            gridMinor = 0x22242a;
            floorColor = 0x17181c;
            accentColor = 0xeab308;
        } else if (theme === 'light') {
            bgColor = 0xf1f5f9;
            gridMajor = 0x0284c7;
            gridMinor = 0xcbd5e1;
            floorColor = 0xffffff;
            accentColor = 0x0284c7;
        }

        // Apply background and fog
        this.scene.background.setHex(bgColor);
        if (this.scene.fog) {
            this.scene.fog.color.setHex(bgColor);
        }

        // Apply floor
        if (this.floor && this.floor.material) {
            this.floor.material.color.setHex(floorColor);
        }

        // Recreate grid with new colors
        if (this.gridHelper) {
            this.scene.remove(this.gridHelper);
            this.gridHelper = new THREE.GridHelper(80, 80, gridMajor, gridMinor);
            this.gridHelper.position.y = 0.01;
            this.scene.add(this.gridHelper);
        }

        // Pedestal ring
        if (this.pedestalRing && this.pedestalRing.material) {
            this.pedestalRing.material.color.setHex(accentColor);
        }

        // Target marker
        if (this.targetMesh && this.targetMesh.material) {
            this.targetMesh.material.color.setHex(accentColor);
            this.targetMesh.material.emissive.setHex(accentColor);
        }
        if (this.targetRing && this.targetRing.material) {
            this.targetRing.material.color.setHex(accentColor);
        }
    }

    buildRobot() {
        if (this.robotGroup) {
            this.scene.remove(this.robotGroup);
        }

        this.robotGroup = new THREE.Group();
        this.scene.add(this.robotGroup);

        // Materials
        this.matDarkMetal = new THREE.MeshStandardMaterial({
            color: 0x1e293b,
            metalness: 0.85,
            roughness: 0.25
        });
        this.matChrome = new THREE.MeshStandardMaterial({
            color: 0xe2e8f0,
            metalness: 0.95,
            roughness: 0.15
        });
        this.matBrass = new THREE.MeshStandardMaterial({
            color: 0xf59e0b,
            metalness: 0.8,
            roughness: 0.3
        });

        // 1. Base Tower (Stationary vertical column)
        const baseColGeo = new THREE.CylinderGeometry(1.2, 1.6, this.baseHeight, 32);
        const baseCol = new THREE.Mesh(baseColGeo, this.matDarkMetal);
        baseCol.position.y = this.baseHeight / 2 + 0.4;
        baseCol.castShadow = true;
        baseCol.receiveShadow = true;
        this.robotGroup.add(baseCol);

        // 2. Base Rotating Turntable (Joint 1: Yaw around Y-axis)
        this.turntable = new THREE.Group();
        this.turntable.position.y = this.baseHeight + 0.4;
        this.robotGroup.add(this.turntable);

        const turnCapGeo = new THREE.CylinderGeometry(1.4, 1.4, 0.4, 32);
        const turnCap = new THREE.Mesh(turnCapGeo, this.matChrome);
        turnCap.position.y = 0.2;
        turnCap.castShadow = true;
        this.turntable.add(turnCap);

        // Shoulder Fork brackets
        const forkGeo = new THREE.BoxGeometry(0.5, 1.4, 1.8);
        const forkL = new THREE.Mesh(forkGeo, this.matDarkMetal);
        forkL.position.set(0.65, 0.9, 0);
        forkL.castShadow = true;
        this.turntable.add(forkL);

        const forkR = new THREE.Mesh(forkGeo, this.matDarkMetal);
        forkR.position.set(-0.65, 0.9, 0);
        forkR.castShadow = true;
        this.turntable.add(forkR);

        // Hierarchical arm links
        this.jointNodes = [];
        this.linkMeshes = [];

        let parentNode = this.turntable;
        const shoulderPivotY = 1.4;

        // Shoulder Pivot point (Joint 2)
        const shoulderNode = new THREE.Group();
        shoulderNode.position.set(0, shoulderPivotY, 0);
        parentNode.add(shoulderNode);
        parentNode = shoulderNode;
        this.jointNodes.push(shoulderNode);

        // Build links dynamically based on robot.dof
        for (let i = 0; i < this.robot.dof; i++) {
            const length = this.robot.links[i];
            const colorHex = this.colors[i % this.colors.length];

            const linkGroup = new THREE.Group();
            parentNode.add(linkGroup);

            // Pivot Bearing Hub (cylinder along Z-axis)
            const hubGeo = new THREE.CylinderGeometry(0.7, 0.7, 1.5, 24);
            const hubMat = this.matDarkMetal;
            const hubMesh = new THREE.Mesh(hubGeo, hubMat);
            hubMesh.rotation.x = Math.PI / 2;
            hubMesh.castShadow = true;
            linkGroup.add(hubMesh);

            // Colored cap on joint bearing
            const capGeo = new THREE.CylinderGeometry(0.45, 0.45, 1.55, 24);
            const capMat = new THREE.MeshStandardMaterial({
                color: colorHex,
                metalness: 0.6,
                roughness: 0.3,
                emissive: colorHex,
                emissiveIntensity: 0.2
            });
            const capMesh = new THREE.Mesh(capGeo, capMat);
            capMesh.rotation.x = Math.PI / 2;
            linkGroup.add(capMesh);

            // Main Arm Bone Link (extends along +X in local space)
            const linkRadius = Math.max(0.25, 0.5 - i * 0.05);
            const boneGeo = new THREE.CylinderGeometry(linkRadius * 0.85, linkRadius, length, 24);
            const boneMat = new THREE.MeshStandardMaterial({
                color: 0x334155,
                metalness: 0.8,
                roughness: 0.3
            });
            const boneMesh = new THREE.Mesh(boneGeo, boneMat);
            boneMesh.rotation.z = -Math.PI / 2;
            boneMesh.position.x = length / 2;
            boneMesh.castShadow = true;
            linkGroup.add(boneMesh);

            // Colored accent sleeve along the arm
            const sleeveGeo = new THREE.CylinderGeometry(linkRadius * 1.05, linkRadius * 1.05, length * 0.6, 24);
            const sleeveMat = new THREE.MeshStandardMaterial({
                color: colorHex,
                metalness: 0.5,
                roughness: 0.25,
                emissive: colorHex,
                emissiveIntensity: 0.15
            });
            const sleeveMesh = new THREE.Mesh(sleeveGeo, sleeveMat);
            sleeveMesh.rotation.z = -Math.PI / 2;
            sleeveMesh.position.x = length / 2;
            sleeveMesh.castShadow = true;
            linkGroup.add(sleeveMesh);

            // Hydraulic / Piston rod decorative detail
            const rodGeo = new THREE.CylinderGeometry(0.08, 0.08, length * 0.7, 16);
            const rodMesh = new THREE.Mesh(rodGeo, this.matChrome);
            rodMesh.rotation.z = -Math.PI / 2;
            rodMesh.position.set(length / 2, linkRadius + 0.15, 0);
            rodMesh.castShadow = true;
            linkGroup.add(rodMesh);

            this.linkMeshes.push({ linkGroup, boneMesh, sleeveMesh, lengthIndex: i });

            // Next joint node at the end of this link
            if (i < this.robot.dof - 1) {
                const nextJointNode = new THREE.Group();
                nextJointNode.position.set(length, 0, 0);
                linkGroup.add(nextJointNode);
                parentNode = nextJointNode;
                this.jointNodes.push(nextJointNode);
            } else {
                // End-Effector Tool Head at the tip
                this.eeMountNode = new THREE.Group();
                this.eeMountNode.position.set(length, 0, 0);
                linkGroup.add(this.eeMountNode);
                this.buildGripper(this.eeMountNode);
            }
        }
    }

    buildGripper(mountNode) {
        this.gripperGroup = new THREE.Group();
        mountNode.add(this.gripperGroup);

        // Tool plate flange
        const flangeGeo = new THREE.CylinderGeometry(0.5, 0.5, 0.25, 24);
        const flange = new THREE.Mesh(flangeGeo, this.matChrome);
        flange.rotation.z = -Math.PI / 2;
        flange.position.x = 0.125;
        this.gripperGroup.add(flange);

        // Gripper palm body
        const palmGeo = new THREE.BoxGeometry(0.6, 0.8, 0.9);
        const palmMat = new THREE.MeshStandardMaterial({ color: 0xef4444, metalness: 0.7, roughness: 0.3 });
        const palm = new THREE.Mesh(palmGeo, palmMat);
        palm.position.x = 0.5;
        palm.castShadow = true;
        this.gripperGroup.add(palm);

        // Gripper Finger Left
        this.fingerL = new THREE.Group();
        this.fingerL.position.set(0.7, 0, 0.35);
        this.gripperGroup.add(this.fingerL);

        const fingerPartGeo = new THREE.BoxGeometry(0.7, 0.18, 0.12);
        const fingerMat = this.matDarkMetal;
        const fL1 = new THREE.Mesh(fingerPartGeo, fingerMat);
        fL1.position.x = 0.35;
        fL1.castShadow = true;
        this.fingerL.add(fL1);

        // Rubber gripping pad Left
        const padGeo = new THREE.BoxGeometry(0.5, 0.14, 0.06);
        const padMat = new THREE.MeshStandardMaterial({ color: 0x10b981, roughness: 0.9 });
        const padL = new THREE.Mesh(padGeo, padMat);
        padL.position.set(0.35, 0, -0.06);
        this.fingerL.add(padL);

        // Gripper Finger Right
        this.fingerR = new THREE.Group();
        this.fingerR.position.set(0.7, 0, -0.35);
        this.gripperGroup.add(this.fingerR);

        const fR1 = new THREE.Mesh(fingerPartGeo, fingerMat);
        fR1.position.x = 0.35;
        fR1.castShadow = true;
        this.fingerR.add(fR1);

        // Rubber gripping pad Right
        const padR = new THREE.Mesh(padGeo, padMat);
        padR.position.set(0.35, 0, 0.06);
        this.fingerR.add(padR);

        // Laser pointer dot at gripper center
        const laserGeo = new THREE.SphereGeometry(0.08, 16, 16);
        const laserMat = new THREE.MeshBasicMaterial({ color: 0xff0000 });
        const laser = new THREE.Mesh(laserGeo, laserMat);
        laser.position.x = 1.2;
        this.gripperGroup.add(laser);

        // Laser beam guide
        const beamGeo = new THREE.CylinderGeometry(0.015, 0.015, 4, 8);
        const beamMat = new THREE.MeshBasicMaterial({ color: 0xff0000, transparent: true, opacity: 0.45 });
        const beam = new THREE.Mesh(beamGeo, beamMat);
        beam.rotation.z = -Math.PI / 2;
        beam.position.x = 3.2;
        this.gripperGroup.add(beam);
    }

    setGripper(val) {
        // val from 0 (closed) to 1 (open)
        this.gripperOpen = Math.max(0, Math.min(1, val));
        if (this.fingerL && this.fingerR) {
            const offset = 0.15 + this.gripperOpen * 0.3;
            this.fingerL.position.z = offset;
            this.fingerR.position.z = -offset;
        }
    }

    buildWorkspaceSphere() {
        const maxReach = this.robot.links.reduce((a, b) => a + b, 0);
        const sphereGeo = new THREE.SphereGeometry(maxReach, 32, 24);
        const sphereMat = new THREE.MeshBasicMaterial({
            color: 0x38bdf8,
            wireframe: true,
            transparent: true,
            opacity: 0.08
        });
        this.workspaceSphere = new THREE.Mesh(sphereGeo, sphereMat);
        this.workspaceSphere.position.set(0, this.baseHeight + 1.8, 0);
        this.scene.add(this.workspaceSphere);
    }

    updateWorkspaceSphere() {
        if (!this.workspaceSphere) return;
        const maxReach = this.robot.links.reduce((a, b) => a + b, 0);
        this.workspaceSphere.scale.set(maxReach, maxReach, maxReach);
        this.workspaceSphere.visible = this.showWorkspace;
    }

    buildTrailRenderer() {
        this.trailGeo = new THREE.BufferGeometry();
        this.trailPositions = new Float32Array(this.maxTrailPoints * 3);
        this.trailGeo.setAttribute('position', new THREE.BufferAttribute(this.trailPositions, 3));

        this.trailMat = new THREE.LineBasicMaterial({
            color: 0xd946ef,
            linewidth: 2.5,
            transparent: true,
            opacity: 0.85
        });
        this.trailLine = new THREE.Line(this.trailGeo, this.trailMat);
        this.scene.add(this.trailLine);
    }

    addTrailPoint(pos) {
        this.trailPoints.push(pos.clone());
        if (this.trailPoints.length > this.maxTrailPoints) {
            this.trailPoints.shift();
        }

        const positions = this.trailPositions;
        for (let i = 0; i < this.trailPoints.length; i++) {
            positions[i * 3] = this.trailPoints[i].x;
            positions[i * 3 + 1] = this.trailPoints[i].y;
            positions[i * 3 + 2] = this.trailPoints[i].z;
        }
        this.trailGeo.setDrawRange(0, this.trailPoints.length);
        this.trailGeo.attributes.position.needsUpdate = true;
    }

    clearTrail() {
        this.trailPoints = [];
        this.trailGeo.setDrawRange(0, 0);
        this.trailGeo.attributes.position.needsUpdate = true;
    }

    setupTransformControls() {
        // Target Visual Mesh (glowing diamond/sphere)
        const targetGeo = new THREE.SphereGeometry(0.35, 24, 24);
        const targetMat = new THREE.MeshStandardMaterial({
            color: 0x38bdf8,
            emissive: 0x0284c7,
            emissiveIntensity: 0.6,
            roughness: 0.2,
            metalness: 0.8
        });
        this.targetMesh = new THREE.Mesh(targetGeo, targetMat);
        this.targetMesh.position.copy(this.target);
        this.scene.add(this.targetMesh);

        // Outer pulsing ring around target
        const ringGeo = new THREE.RingGeometry(0.6, 0.7, 32);
        const ringMat = new THREE.MeshBasicMaterial({ color: 0x38bdf8, side: THREE.DoubleSide, transparent: true, opacity: 0.6 });
        this.targetRing = new THREE.Mesh(ringGeo, ringMat);
        this.targetMesh.add(this.targetRing);

        // Three.js TransformControls Gizmo
        if (THREE.TransformControls) {
            this.transformGizmo = new THREE.TransformControls(this.camera, this.renderer.domElement);
            this.transformGizmo.size = 0.85;
            this.transformGizmo.attach(this.targetMesh);
            this.scene.add(this.transformGizmo);

            // Disable OrbitControls while dragging gizmo
            this.transformGizmo.addEventListener('dragging-changed', (event) => {
                this.controls.enabled = !event.value;
            });

            // Dragging callback: update target & solve 3D IK!
            this.transformGizmo.addEventListener('change', () => {
                this.target.copy(this.targetMesh.position);
                if (this.onTargetChange) {
                    this.onTargetChange(this.target.x, this.target.y, this.target.z);
                }
                this.solve3DIK(this.target.x, this.target.y, this.target.z);
            });
        }
    }

    setupEventListeners() {
        window.addEventListener('resize', () => this.resize());
        if (typeof ResizeObserver !== 'undefined') {
            new ResizeObserver(() => this.resize()).observe(this.container);
        }
    }

    resize() {
        const width = this.container.clientWidth || 800;
        const height = this.container.clientHeight || 600;
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height);
    }

    /**
     * 3D Forward Kinematics update to Three.js scene graph
     */
    updateRobotPose() {
        if (!this.jointNodes || this.jointNodes.length === 0) return;

        // Base Yaw angle: robot.angles[0] rotates turntable around Y-axis
        const yawDeg = this.robot.angles[0];
        this.turntable.rotation.y = (yawDeg * Math.PI) / 180;

        // Remaining joint pitch angles: rotate around local Z-axis
        for (let i = 1; i < this.robot.dof; i++) {
            const pitchDeg = this.robot.angles[i];
            const node = this.jointNodes[i - 1];
            if (node) {
                node.rotation.z = (pitchDeg * Math.PI) / 180;
            }
        }

        // Get world position of End-Effector
        if (this.eeMountNode) {
            const eeWorldPos = new THREE.Vector3();
            this.eeMountNode.getWorldPosition(eeWorldPos);

            if (this.showTrail) {
                this.addTrailPoint(eeWorldPos);
            }
        }

        this.setGripper(this.gripperOpen);
    }

    /**
     * 3D Inverse Kinematics
     * Given (X, Y, Z):
     * 1. Base Yaw theta_1 = atan2(Z, X)
     * 2. Planar projection: R = sqrt(X^2 + Z^2), Height = Y - basePivotHeight
     * 3. Solve 2D pitch angles using robot.solveFABRIK or CCD
     */
    solve3DIK(targetX, targetY, targetZ) {
        // Base yaw
        let yawRad = Math.atan2(targetZ, targetX);
        let yawDeg = (yawRad * 180) / Math.PI;

        // Planar reach
        const planarR = Math.hypot(targetX, targetZ);
        const pivotY = this.baseHeight + 1.8;
        const planarY = targetY - pivotY;

        // Solve remaining angles using planar solver
        const planarLengths = this.robot.links.slice(1);
        const savedAngles = [...this.robot.angles];

        // Temporary 2D robot with remaining links
        const tempRobot = new RobotKinematics(
            planarLengths.length > 0 ? planarLengths : [5, 4, 3],
            savedAngles.slice(1)
        );
        tempRobot.enableLimits = this.robot.enableLimits;

        const ikRes = tempRobot.solveFABRIK(planarR, planarY, 35, 0.005);

        // Combine solved angles: [yawDeg, ...tempRobot.angles]
        const newAngles = [yawDeg];
        for (let i = 0; i < this.robot.dof - 1; i++) {
            newAngles.push(tempRobot.angles[i] !== undefined ? tempRobot.angles[i] : 0);
        }

        this.robot.angles = newAngles;
        this.updateRobotPose();

        if (this.onAnglesChange) {
            this.onAnglesChange(this.robot.angles);
        }

        return ikRes;
    }

    /**
     * Camera View Presets
     */
    setCameraPreset(name) {
        const center = new THREE.Vector3(0, 4, 0);
        this.controls.target.copy(center);

        if (name === 'iso') {
            this.camera.position.set(16, 14, 20);
        } else if (name === 'front') {
            this.camera.position.set(0, 5, 24);
        } else if (name === 'side') {
            this.camera.position.set(24, 5, 0);
        } else if (name === 'top') {
            this.camera.position.set(0, 26, 0.1);
        } else if (name === 'ee') {
            if (this.eeMountNode) {
                const eePos = new THREE.Vector3();
                this.eeMountNode.getWorldPosition(eePos);
                this.controls.target.copy(eePos);
                this.camera.position.set(eePos.x + 5, eePos.y + 4, eePos.z + 5);
            }
        }
        this.controls.update();
    }

    /**
     * Main 3D Animation Loop
     */
    animate() {
        requestAnimationFrame(() => this.animate());

        // Subtle animation on target ring
        if (this.targetRing) {
            this.targetRing.rotation.z += 0.02;
            this.targetRing.lookAt(this.camera.position);
        }

        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }
}

// Export for browser
if (typeof window !== 'undefined') {
    window.Robot3DVisualizer = Robot3DVisualizer;
}
