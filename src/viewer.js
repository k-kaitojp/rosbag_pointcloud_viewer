// Three.js point cloud viewer: scene, camera, orbit controls, grid/axes,
// and colorization of points by axis / intensity / rgb / flat.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// Turbo-ish colormap approximation for scalar colorization.
function colormap(t) {
  // t in [0,1] -> [r,g,b] 0..1, a simple blue->cyan->green->yellow->red ramp
  const c = Math.min(1, Math.max(0, t));
  const r = Math.min(1, Math.max(0, 1.5 - Math.abs(4 * c - 3)));
  const g = Math.min(1, Math.max(0, 1.5 - Math.abs(4 * c - 2)));
  const b = Math.min(1, Math.max(0, 1.5 - Math.abs(4 * c - 1)));
  return [r, g, b];
}

export class PointCloudViewer {
  constructor(container) {
    this.container = container;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0d1117);

    const { clientWidth: w, clientHeight: h } = container;
    this.camera = new THREE.PerspectiveCamera(60, w / h, 0.01, 100000);
    this.camera.position.set(8, 8, 8);
    this.camera.up.set(0, 0, 1); // ROS convention: Z up

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setSize(w, h);
    container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.1;

    // Reference grid (XY plane) and axes (ROS: X red, Y green, Z blue).
    this.grid = new THREE.GridHelper(20, 20, 0x335577, 0x223344);
    this.grid.rotation.x = Math.PI / 2; // GridHelper is XZ by default -> make it XY
    this.scene.add(this.grid);
    this.axes = new THREE.AxesHelper(2);
    this.scene.add(this.axes);

    this.geometry = new THREE.BufferGeometry();
    this.material = new THREE.PointsMaterial({
      size: 0.03,
      vertexColors: true,
      sizeAttenuation: true,
    });
    this.points = new THREE.Points(this.geometry, this.material);
    this.scene.add(this.points);

    this.pointSize = 0.03;
    this.colorMode = 'axis-z';
    this.flatColor = new THREE.Color(0x4fc3f7);
    this.current = null; // last extracted cloud

    this._onResize = () => this.resize();
    window.addEventListener('resize', this._onResize);

    this._animate = this._animate.bind(this);
    this.renderer.setAnimationLoop(this._animate);
  }

  _animate() {
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  resize() {
    const { clientWidth: w, clientHeight: h } = this.container;
    if (w === 0 || h === 0) return;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  }

  setPointSize(size) {
    this.pointSize = size;
    this.material.size = size;
  }

  setColorMode(mode) {
    this.colorMode = mode;
    if (this.current) this._applyColors(this.current);
  }

  /** Color field options available for the current cloud. */
  availableColorModes(extracted) {
    const modes = [
      { value: 'axis-z', label: 'Height (Z)' },
      { value: 'axis-x', label: 'Axis X' },
      { value: 'axis-y', label: 'Axis Y' },
      { value: 'flat', label: 'Flat color' },
    ];
    if (extracted.rgb) modes.unshift({ value: 'rgb', label: 'RGB (embedded)' });
    for (const name of Object.keys(extracted.scalars)) {
      modes.push({ value: `scalar:${name}`, label: `Field: ${name}` });
    }
    return modes;
  }

  showCloud(extracted) {
    this.current = extracted;
    this.geometry.setAttribute(
      'position',
      new THREE.BufferAttribute(extracted.positions, 3),
    );
    this._applyColors(extracted);
    this.geometry.computeBoundingSphere();
  }

  _applyColors(extracted) {
    const { count, positions, bounds, scalars, rgb } = extracted;
    const colors = new Float32Array(count * 3);
    const mode = this.colorMode;

    if (mode === 'rgb' && rgb) {
      colors.set(rgb);
    } else if (mode === 'flat') {
      const { r, g, b } = this.flatColor;
      for (let i = 0; i < count; i++) {
        colors[i * 3] = r;
        colors[i * 3 + 1] = g;
        colors[i * 3 + 2] = b;
      }
    } else if (mode.startsWith('scalar:')) {
      const name = mode.slice('scalar:'.length);
      const arr = scalars[name];
      this._rampColors(colors, count, arr, (i) => arr[i]);
    } else {
      // axis-x / axis-y / axis-z
      const axis = mode === 'axis-x' ? 0 : mode === 'axis-y' ? 1 : 2;
      const lo = bounds.min[axis];
      const hi = bounds.max[axis];
      const span = hi - lo || 1;
      for (let i = 0; i < count; i++) {
        const t = (positions[i * 3 + axis] - lo) / span;
        const [r, g, b] = colormap(t);
        colors[i * 3] = r;
        colors[i * 3 + 1] = g;
        colors[i * 3 + 2] = b;
      }
    }

    this.geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    this.geometry.attributes.color.needsUpdate = true;
  }

  _rampColors(colors, count, arr) {
    let lo = Infinity;
    let hi = -Infinity;
    for (let i = 0; i < count; i++) {
      const v = arr[i];
      if (!Number.isFinite(v)) continue;
      if (v < lo) lo = v;
      if (v > hi) hi = v;
    }
    const span = hi - lo || 1;
    for (let i = 0; i < count; i++) {
      const t = (arr[i] - lo) / span;
      const [r, g, b] = colormap(t);
      colors[i * 3] = r;
      colors[i * 3 + 1] = g;
      colors[i * 3 + 2] = b;
    }
  }

  /** Position camera to frame the current cloud (call once per topic load). */
  frameCloud(bounds) {
    const { center, size } = bounds;
    this.controls.target.set(center[0], center[1], center[2]);
    const dist = size * 1.8 + 1;
    this.camera.position.set(
      center[0] + dist,
      center[1] + dist,
      center[2] + dist * 0.8,
    );
    this.camera.near = Math.max(size / 1000, 0.01);
    this.camera.far = size * 100 + 1000;
    this.camera.updateProjectionMatrix();

    // Resize grid to roughly match the scene scale.
    const gridSize = Math.max(10, Math.ceil(size));
    this.scene.remove(this.grid);
    this.grid = new THREE.GridHelper(gridSize * 2, 20, 0x335577, 0x223344);
    this.grid.rotation.x = Math.PI / 2;
    this.scene.add(this.grid);

    this.controls.update();
  }

  dispose() {
    window.removeEventListener('resize', this._onResize);
    this.renderer.setAnimationLoop(null);
    this.geometry.dispose();
    this.material.dispose();
    this.renderer.dispose();
  }
}
