<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref } from 'vue'
import { request } from './api'
import { useDevice } from './device'

const device = useDevice()
const stopMessage = ref('')
const stopping = ref(false)
const robot = computed(() => device.fresh ? device.snapshot?.connection.robot : 'offline')
const robotLabel = computed(() => ({ online: '在线', diagnostic: '诊断', offline: '离线' })[robot.value ?? 'offline'])
const wristOnline = computed(() => device.fresh && device.snapshot?.connection.wearable === 'online')
const navigation = [
  ['/', '总览'], ['/manual', '手动调试'], ['/teaching', '示教录制'],
  ['/replay', '轨迹回放'], ['/settings', '设备标定'],
]

async function softwareStop() {
  if (stopping.value) return
  stopping.value = true
  stopMessage.value = ''
  try {
    await request('/api/control/stop', {})
    stopMessage.value = '服务端已确认软件停止。'
  } catch (error) {
    stopMessage.value = `${error instanceof Error ? error.message : '停止请求失败'}。需要紧急停止时请断开舵机动力。`
  } finally {
    stopping.value = false
  }
}

onMounted(device.start)
onBeforeUnmount(device.stop)
</script>

<template>
  <header class="console-header">
    <div class="shell header-row">
      <RouterLink to="/" class="product">
        <span class="product-mark" aria-hidden="true"><i></i><i></i><i></i></span>
        <span>MotionArm<small>机械臂控制台</small></span>
      </RouterLink>
      <div class="header-status" aria-label="设备状态">
        <span><i class="dot" :class="{ lit: robot !== 'offline' }"></i> 执行端 <b>{{ robotLabel }}</b></span>
        <span><i class="dot" :class="{ lit: wristOnline }"></i> 手腕端 <b>{{ wristOnline ? '在线' : '离线' }}</b></span>
        <span v-if="device.snapshot?.source === 'mock'" class="simulation-tag">软件模拟</span>
      </div>
      <button class="stop-button" :disabled="stopping || !device.fresh || robot === 'offline'"
        @click="softwareStop">■ {{ stopping ? '请求停止中' : '软件停止' }}</button>
    </div>
  </header>
  <div class="nav-border">
    <div class="shell nav-row">
      <nav aria-label="工作区">
        <RouterLink v-for="[path, name] in navigation" :key="path" :to="path">{{ name }}</RouterLink>
      </nav>
      <span class="physical-stop">紧急停止请断开舵机独立动力</span>
    </div>
  </div>
  <main class="shell main-shell">
    <p v-if="stopMessage" class="notice" aria-live="polite">{{ stopMessage }}</p>
    <p v-if="!device.fresh" class="notice warning" aria-live="polite">
      {{ device.connection === 'connecting' ? '正在连接遥测服务。' : '遥测已断开或超时，动作入口已锁定。' }}
      <span v-if="device.connection !== 'connecting'">检查本地 FastAPI 服务。连接恢复后可重新查询状态。</span>
    </p>
    <p v-else-if="device.snapshot?.read_only" class="notice warning" role="status">
      执行端只读。完成实物行程标定和固件动作验收后才能启用控制、录制和回放。
      <span v-if="device.snapshot.connection.detail">{{ device.snapshot.connection.detail }}</span>
    </p>
    <RouterView />
  </main>
</template>
