<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref, watch } from 'vue'
import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { fallbackGeometry, tipPosition } from './geometry'
import { sceneResizeAction, type SceneSize } from './displayStability'
import type { Geometry, Pose } from './types'

const props = defineProps<{ pose: Pose; requested: Pose; geometry?: Geometry; muted: boolean; history: boolean; ghost: boolean; axes: boolean; dimensions: boolean; reference?: boolean }>()
const host = ref<HTMLDivElement>()
const error = ref('')
const view = ref('iso')
const tip = computed(() => tipPosition(props.pose, props.geometry ?? fallbackGeometry))
let renderer: THREE.WebGLRenderer | undefined
let scene: THREE.Scene, camera: THREE.PerspectiveCamera, controls: OrbitControls
let frame = 0, resize: ResizeObserver | undefined
let fitCount = 0
let lastSize: SceneSize | null = null
let body: ReturnType<typeof makeArm>, ghostBody: ReturnType<typeof makeArm>
const materials: Array<{ material: THREE.MeshStandardMaterial; color: number }> = []
const axisHelpers: THREE.AxesHelper[] = []
const dimensions: THREE.Group[] = []
function contextLost(event: Event) {
  event.preventDefault()
  error.value = 'WebGL 上下文已中断，正在等待恢复；也可刷新页面重试。'
}
function contextRestored() { error.value = '' }

function material(color: number, ghost = false) {
  const mat = new THREE.MeshStandardMaterial({ color: ghost ? 0x55d8ee : color, roughness: 0.44, metalness: 0.65,
    transparent: ghost, opacity: ghost ? 0.24 : 1, wireframe: ghost, depthWrite: !ghost })
  if (!ghost) materials.push({ material: mat, color })
  return mat
}
function box(parent: THREE.Object3D, size: number[], pos: number[], mat: THREE.Material) {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(size[0], size[1], size[2]), mat)
  mesh.position.set(pos[0]!, pos[1]!, pos[2]!)
  mesh.castShadow = true; mesh.receiveShadow = true; parent.add(mesh)
  return mesh
}
function cylinder(parent: THREE.Object3D, radius: number, length: number, pos: number[], mat: THREE.Material, axis = 'z') {
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, length, 40), mat)
  if (axis === 'z') mesh.rotation.x = Math.PI / 2
  mesh.position.set(pos[0]!, pos[1]!, pos[2]!)
  mesh.castShadow = true; mesh.receiveShadow = true; parent.add(mesh)
  return mesh
}
function label(text: string, pos: number[]) {
  const canvas = document.createElement('canvas'); canvas.width = 384; canvas.height = 72
  const ctx = canvas.getContext('2d')!
  ctx.fillStyle = 'rgba(8, 23, 33, 0.9)'; ctx.fillRect(0, 0, 384, 72)
  ctx.font = '500 30px sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'
  ctx.fillStyle = '#b9d6e4'; ctx.fillText(text, 192, 38)
  const texture = new THREE.CanvasTexture(canvas)
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, depthTest: false }))
  sprite.position.set(pos[0]!, pos[1]!, pos[2]!); sprite.scale.set(86, 16, 1)
  return sprite
}
function dimension(parent: THREE.Object3D, length: number, offset: number, text: string) {
  const group = new THREE.Group()
  const points = [new THREE.Vector3(offset - 5, 0, 0), new THREE.Vector3(offset + 5, 0, 0), new THREE.Vector3(offset, 0, 0),
    new THREE.Vector3(offset, 0, length), new THREE.Vector3(offset - 5, 0, length), new THREE.Vector3(offset + 5, 0, length)]
  group.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(points), new THREE.LineBasicMaterial({ color: 0x4c7c91 })))
  group.add(label(text, [offset - 30, 0, length / 2])); parent.add(group); dimensions.push(group)
}
function makeArm(ghost = false) {
  const g = props.geometry ?? fallbackGeometry
  const root = new THREE.Group()
  const shell = material(0x173541, ghost), plate = material(0x384d58, ghost)
  const dark = material(0x101a23, ghost), steel = material(0x889ba4, ghost), accent = material(0x45c5c1, ghost)
  const orange = material(0xdc8e49, ghost)
  box(root, [g.base_width, 108, 7], [0, 0, 4], plate)
  for (const x of [-35, 35]) for (const y of [-42, 42]) cylinder(root, 3, 9, [x, y, 7], steel)
  cylinder(root, 35, 24, [0, 0, 20], dark)
  const yaw = new THREE.Group(); root.add(yaw)
  cylinder(yaw, 37, 7, [0, 0, 36], shell)
  box(yaw, [34, 39, 36], [0, 0, 58], dark)
  for (const side of [-1, 1]) box(yaw, [36, 5, 48], [0, side * 25, 65], plate)
  cylinder(yaw, 17, 59, [0, 0, g.table_to_shoulder_axis], steel, 'y')
  const shoulder = new THREE.Group(); shoulder.position.z = g.table_to_shoulder_axis; yaw.add(shoulder)
  for (const side of [-1, 1]) {
    box(shoulder, [22, 5, g.shoulder_to_elbow_axis], [0, side * 25, g.shoulder_to_elbow_axis / 2], shell)
    box(shoulder, [9, 2, g.shoulder_to_elbow_axis - 26], [0, side * 28, g.shoulder_to_elbow_axis / 2], accent)
    for (const z of [11, g.shoulder_to_elbow_axis - 11]) cylinder(shoulder, 4, 8, [0, side * 27, z], steel, 'y')
  }
  box(shoulder, [29, 42, 39], [0, 0, g.shoulder_to_elbow_axis - 12], dark)
  box(shoulder, [20, 2, 12], [0, -23, g.shoulder_to_elbow_axis - 10], orange)
  cylinder(shoulder, 15, 61, [0, 0, g.shoulder_to_elbow_axis], steel, 'y')
  const elbow = new THREE.Group(); elbow.position.z = g.shoulder_to_elbow_axis; shoulder.add(elbow)
  for (const side of [-1, 1]) box(elbow, [25, 5, g.elbow_to_gripper_mount], [0, side * 20, g.elbow_to_gripper_mount / 2], shell)
  box(elbow, [25, 38, 24], [0, 0, g.elbow_to_gripper_mount - 10], dark)
  box(elbow, [32, 49, 8], [0, 0, g.elbow_to_gripper_mount], plate)
  const mount = new THREE.Group(); mount.position.z = g.elbow_to_gripper_mount; elbow.add(mount)
  box(mount, [32, 34, 23], [0, 0, 14], dark)
  cylinder(mount, 9, 40, [0, 0, 26], steel, 'y')
  const fingers: THREE.Group[] = []
  for (const side of [-1, 1]) {
    const finger = new THREE.Group(); finger.position.set(0, side * 17, 25); mount.add(finger)
    const length = g.gripper_mount_to_tip - 25
    box(finger, [13, 5, length - 8], [0, 0, (length - 8) / 2], steel)
    box(finger, [13, 12, 8], [0, -side * 3, length - 4], accent)
    fingers.push(finger)
  }
  if (!ghost) {
    for (const parent of [root, shoulder, elbow]) { const axes = new THREE.AxesHelper(43); parent.add(axes); axisHelpers.push(axes) }
    dimension(shoulder, g.shoulder_to_elbow_axis, -45, g.shoulder_to_elbow_axis + ' mm')
    dimension(elbow, g.elbow_to_gripper_mount + g.gripper_mount_to_tip, -42, g.elbow_to_gripper_mount + ' + ' + g.gripper_mount_to_tip + ' mm')
    dimension(root, g.table_to_shoulder_axis, -65, '轴高 ' + g.table_to_shoulder_axis)
  }
  scene.add(root)
  return { root, yaw, shoulder, elbow, fingers }
}
function applyPose(arm: ReturnType<typeof makeArm>, pose: Pose) {
  arm.yaw.rotation.z = THREE.MathUtils.degToRad(pose.base_yaw_deg)
  arm.shoulder.rotation.y = THREE.MathUtils.degToRad(pose.shoulder_down_deg)
  arm.elbow.rotation.y = THREE.MathUtils.degToRad(pose.elbow_down_deg)
  arm.fingers[0]!.position.y = -8 - pose.gripper_open_ratio * 18
  arm.fingers[1]!.position.y = 8 + pose.gripper_open_ratio * 18
}
function fitModel() {
  if (!camera || !body) return
  // Observable for browser regressions; telemetry must never increment this.
  if (host.value) host.value.dataset.cameraFitCount = String(++fitCount)
  applyPose(body, props.pose)
  body.root.updateMatrixWorld(true)
  const bounds = new THREE.Box3()
  body.root.traverse((node) => { if (node instanceof THREE.Mesh) bounds.expandByObject(node) })
  const sphere = bounds.getBoundingSphere(new THREE.Sphere())
  const direction = camera.position.clone().sub(controls.target).normalize()
  const vertical = THREE.MathUtils.degToRad(camera.fov / 2)
  const horizontal = Math.atan(Math.tan(vertical) * camera.aspect)
  const distance = sphere.radius * 1.2 / Math.sin(Math.min(vertical, horizontal))
  controls.target.copy(sphere.center)
  camera.position.copy(sphere.center).addScaledVector(direction, distance)
  controls.update()
}
function preset(name: string) {
  if (name === 'fit') { fitModel(); return }
  view.value = name
  if (!camera) return
  const positions: Record<string, number[]> = { iso: [490, -650, 420], front: [0, -720, 190], side: [720, 0, 190], top: [100, -0.1, 850] }
  const pos = positions[name] ?? positions.iso!
  camera.position.set(pos[0]!, pos[1]!, pos[2]!); controls.target.set(75, 0, 145); fitModel()
}
onMounted(() => {
  try {
    scene = new THREE.Scene(); scene.background = new THREE.Color(0x0a141e)
    scene.fog = new THREE.Fog(0x0a141e, 900, 1800)
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false })
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
    renderer.shadowMap.enabled = true; renderer.shadowMap.type = THREE.PCFShadowMap
    renderer.domElement.addEventListener('webglcontextlost', contextLost)
    renderer.domElement.addEventListener('webglcontextrestored', contextRestored)
    renderer.toneMapping = THREE.ACESFilmicToneMapping; renderer.toneMappingExposure = 1.45
    host.value!.appendChild(renderer.domElement)
    camera = new THREE.PerspectiveCamera(38, 1, 1, 3000); camera.up.set(0, 0, 1)
    controls = new OrbitControls(camera, renderer.domElement)
    controls.enableDamping = true; controls.dampingFactor = 0.08
    controls.minDistance = 200; controls.maxDistance = 2400; controls.maxPolarAngle = Math.PI * 0.499
    scene.add(new THREE.HemisphereLight(0xc1efff, 0x182530, 2.5))
    const key = new THREE.DirectionalLight(0xdaf6ff, 4); key.position.set(150, -200, 550); key.castShadow = true
    key.shadow.mapSize.set(2048, 2048); Object.assign(key.shadow.camera, { left: -400, right: 400, top: 400, bottom: -400 }); key.shadow.bias = -0.001; scene.add(key)
    const rim = new THREE.DirectionalLight(0x3eaaa9, 3); rim.position.set(-200, 200, 300); scene.add(rim)
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(2200, 2200), new THREE.MeshStandardMaterial({ color: 0x101c27, roughness: 0.96 }))
    floor.position.z = -1; floor.receiveShadow = true; scene.add(floor)
    const grid = new THREE.GridHelper(1400, 28, 0x2e4b5a, 0x1b303e); grid.rotation.x = Math.PI / 2; grid.position.z = 0.1; scene.add(grid)
    const circle = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(Array.from({ length: 128 }, (_, i) => new THREE.Vector3(Math.cos(i / 128 * Math.PI * 2) * 300, Math.sin(i / 128 * Math.PI * 2) * 300, 0.3))), new THREE.LineDashedMaterial({ color: 0x315362, dashSize: 8, gapSize: 5 }))
    circle.computeLineDistances(); scene.add(circle)
    body = makeArm(); ghostBody = makeArm(true); preset('iso')
    const measure = () => {
      if (!host.value || !renderer) return
      const { width, height } = host.value.getBoundingClientRect()
      if (width <= 0 || height <= 0) return
      const size = { width, height, viewportWidth: window.innerWidth, viewportHeight: window.innerHeight }
      const action = sceneResizeAction(lastSize, size)
      lastSize = size
      if (action === 'none') return
      camera.aspect = width / height
      camera.updateProjectionMatrix()
      renderer.setSize(width, height)
      if (action === 'fit') fitModel()
    }
    resize = new ResizeObserver(measure); resize.observe(host.value!); measure()
    const draw = () => {
      frame = requestAnimationFrame(draw)
      applyPose(body, props.pose); applyPose(ghostBody, props.requested)
      ghostBody.root.visible = props.ghost && !props.muted && !props.history
      for (const entry of materials) entry.material.color.setHex(props.muted ? 0x52616b : entry.color)
      for (const axes of axisHelpers) axes.visible = props.axes
      for (const group of dimensions) group.visible = props.dimensions
      controls.update(); if (!error.value) renderer!.render(scene, camera)
    }; draw()
  } catch (cause) {
    error.value = cause instanceof Error ? cause.message : 'WebGL 初始化失败'
  }
})
watch(() => props.geometry, () => { /* Geometry is immutable within a scene instance; parent keys it by baseline. */ })
onBeforeUnmount(() => {
  cancelAnimationFrame(frame); resize?.disconnect(); controls?.dispose()
  renderer?.domElement.removeEventListener('webglcontextlost', contextLost)
  renderer?.domElement.removeEventListener('webglcontextrestored', contextRestored)
  scene?.traverse((node) => {
    if (node instanceof THREE.Mesh || node instanceof THREE.Line || node instanceof THREE.Sprite) {
      if ('geometry' in node) node.geometry.dispose()
      for (const mat of Array.isArray(node.material) ? node.material : [node.material]) {
        if ('map' in mat && mat.map instanceof THREE.Texture) mat.map.dispose()
        mat.dispose()
      }
    }
  })
  renderer?.dispose()
})
</script>

<template>
  <div class="scene-frame">
    <div ref="host" class="scene-canvas" :class="{ unavailable: error }" role="img" aria-label="机械臂虚拟关节目标三维估算预览"></div>
    <div v-if="error" class="webgl-error"><span>◇</span><strong>三维视图不可用</strong><p>三维渲染暂不可用，数值状态台仍可使用。</p><small>{{ error }}</small></div>
    <div class="scene-title"><span class="eyebrow">{{ history ? 'RECORDED MOTION' : 'VIRTUAL EXECUTOR' }}</span><strong>{{ history ? '历史回放' : '虚拟关节目标／估算预览' }}</strong><span v-if="muted && !history" class="scene-stale">{{ reference ? '参考姿态 · 尚无有效输入' : '目标已保持 · 实时输入失效' }}</span></div>
    <div class="camera-presets" aria-label="三维视角">
      <button v-for="[key, label] in [['iso', '等轴'], ['front', '正面'], ['side', '侧面'], ['top', '俯视'], ['fit', '适配模型']]" :key="key" :class="{ active: view === key }" @click="preset(key!)">{{ label }}</button>
    </div>
    <div class="scene-compass"><span class="axis-x">X</span><span class="axis-y">Y</span><span class="axis-z">Z ↑</span><small>世界坐标 / mm</small></div>
    <div class="tip-readout"><span>爪尖估算</span><b>X {{ tip.x.toFixed(0) }}</b><b>Y {{ tip.y.toFixed(0) }}</b><b>Z {{ tip.z.toFixed(0) }}</b><small>mm</small></div>
    <div class="scene-help">拖动旋转 · 滚轮缩放 · 右键平移</div>
  </div>
</template>
