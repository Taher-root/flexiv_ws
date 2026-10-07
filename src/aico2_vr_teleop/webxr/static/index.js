var polyfill = new WebXRPolyfill();
import * as THREE from 'three';

var messageData = null;
const camTextures = {};
const camMeshes = {};
let poseCount = 0;

function logStatus(msg) {
    const el = document.getElementById('log');
    if (el) {
        el.textContent = msg;
        el.style.display = 'block';
    }
    console.log('[VR]', msg);
}

async function activateXR() {
    logStatus('Checking WebXR support...');

    if (!navigator.xr) {
        logStatus('ERROR: WebXR not supported in this browser');
        return;
    }

    const arSupported = await navigator.xr.isSessionSupported("immersive-ar");
    const vrSupported = await navigator.xr.isSessionSupported("immersive-vr");
    logStatus('AR: ' + arSupported + ', VR: ' + vrSupported);

    const canvas = document.createElement("canvas");
    document.body.appendChild(canvas);
    const gl = canvas.getContext("webgl", { xrCompatible: true });

    const scene = new THREE.Scene();

    const indicatorLeft = new THREE.Mesh(
        new THREE.SphereGeometry(0.015, 12, 8),
        new THREE.MeshBasicMaterial({ color: 0x00aaff })
    );
    indicatorLeft.visible = false;
    scene.add(indicatorLeft);

    const indicatorRight = new THREE.Mesh(
        new THREE.SphereGeometry(0.015, 12, 8),
        new THREE.MeshBasicMaterial({ color: 0x00ff00 })
    );
    indicatorRight.visible = false;
    scene.add(indicatorRight);

    const indicators = { left: indicatorLeft, right: indicatorRight };

    createCamPlane(scene, 'left',  -0.22, 0.15, -0.7);
    createCamPlane(scene, 'right',  0.22, 0.15, -0.7);
    createCamPlane(scene, 'head',   0.00, -0.12, -0.7);

    const renderer = new THREE.WebGLRenderer({
        alpha: true,
        preserveDrawingBuffer: true,
        canvas: canvas,
        context: gl,
    });

    const camera = new THREE.PerspectiveCamera();
    camera.matrixAutoUpdate = false;

    const mode = arSupported ? "immersive-ar" : "immersive-vr";
    logStatus('Requesting ' + mode + ' session...');

    let session;
    try {
        session = await navigator.xr.requestSession(mode);
    } catch (e) {
        logStatus('ERROR: ' + e.message);
        return;
    }
    logStatus(mode + ' session started');

    session.updateRenderState({
        baseLayer: new XRWebGLLayer(session, gl),
    });

    const referenceSpace = await session.requestReferenceSpace('local');

    setInterval(fetchCamFrames, 200);

    const onXRFrame = (time, frame) => {
        session.requestAnimationFrame(onXRFrame);
        gl.bindFramebuffer(gl.FRAMEBUFFER, session.renderState.baseLayer.framebuffer);

        const pose = frame.getViewerPose(referenceSpace);
        if (pose) {
            const view = pose.views[0];
            const viewport = session.renderState.baseLayer.getViewport(view);
            renderer.setSize(viewport.width, viewport.height);

            camera.matrix.fromArray(view.transform.matrix);
            camera.projectionMatrix.fromArray(view.projectionMatrix);
            camera.updateMatrixWorld(true);

            renderer.render(scene, camera);
        }

        collectControllerData(session, frame, referenceSpace, indicators);
    };
    session.requestAnimationFrame(onXRFrame);
}

// ---- Camera feed helpers ----

function createCamPlane(scene, name, x, y, z) {
    const geo = new THREE.PlaneGeometry(0.28, 0.21);
    const tex = new THREE.Texture();
    tex.minFilter = THREE.LinearFilter;
    const mat = new THREE.MeshBasicMaterial({ map: tex, side: THREE.DoubleSide });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set(x, y, z);
    mesh.visible = false;
    scene.add(mesh);
    camTextures[name] = tex;
    camMeshes[name] = mesh;
}

async function fetchCamFrames() {
    for (const name of Object.keys(camTextures)) {
        try {
            const resp = await fetch('/cam/' + name + '/snapshot');
            if (!resp.ok) continue;
            const blob = await resp.blob();
            const bmp = await createImageBitmap(blob);
            camTextures[name].image = bmp;
            camTextures[name].needsUpdate = true;
            camMeshes[name].visible = true;
        } catch (e) {
            // camera not ready yet
        }
    }
}

// ---- Pose sending ----

let lastTriggerStr = '';

function sendBatch(batch) {
    poseCount++;
    if (poseCount % 30 === 0) {
        logStatus('Frames: ' + poseCount + ' | ' + lastTriggerStr);
    }
    fetch('/pose', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(batch),
    })
    .then(r => r.json())
    .then(d => { messageData = d; })
    .catch(() => {});
}

// ---- Controller data ----

let controllerWarnCount = 0;

function collectControllerData(session, frame, refSpace, indicators) {
    if (session.inputSources.length === 0) {
        controllerWarnCount++;
        if (controllerWarnCount % 60 === 1) {
            logStatus('No controllers detected (' + controllerWarnCount + ')');
        }
        return;
    }

    const batch = [];

    for (const source of session.inputSources) {
        if (!source.gamepad) continue;

        const hand = source.handedness || source.gamepad.hand;
        if (hand !== 'left' && hand !== 'right') continue;

        const pose = frame.getPose(source.gripSpace, refSpace);
        if (!pose) continue;

        const gp = source.gamepad;
        const backTrigger = gp.buttons[1].value;

        const ind = indicators[hand];
        if (ind) {
            ind.visible = backTrigger > 0.95;
            if (ind.visible) {
                const p = pose.transform.position;
                ind.position.set(p.x, p.y + 0.06, p.z);
            }
        }

        const pos = pose.transform.position;
        const ori = pose.transform.orientation;

        lastTriggerStr = hand[0] + ':' + backTrigger.toFixed(2);

        batch.push({
            position:    { x: pos.x,  y: pos.y,  z: pos.z  },
            orientation: { x: ori.x,  y: ori.y,  z: ori.z,  w: ori.w  },
            hand:             hand,
            triggerValue:     gp.buttons[0].value,
            backTriggerValue: backTrigger,
            buttonAValue:     gp.buttons[4].value,
            buttonBValue:     gp.buttons[5].value,
            timestamp:        Date.now(),
        });
    }

    if (batch.length > 0) {
        sendBatch(batch);
    }
}

// ---- Start button ----

document.querySelector('#start-button').addEventListener('click', () => {
    activateXR();
});
