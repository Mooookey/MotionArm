<script setup lang="ts">
import { onBeforeUnmount, onMounted, ref, watch } from 'vue'
import { init, use, type EChartsType } from 'echarts/core'
import { LineChart } from 'echarts/charts'
import { GridComponent, LegendComponent, TooltipComponent } from 'echarts/components'
import { CanvasRenderer } from 'echarts/renderers'
import type { Point } from './types'

use([LineChart, GridComponent, LegendComponent, TooltipComponent, CanvasRenderer])
const props = defineProps<{ points: Point[]; type: 'imu' | 'joints'; live?: boolean }>()
const node = ref<HTMLDivElement>()
const palettes = ['#1677a7', '#c08731', '#6d5aa6', '#138675']
const labels = ['底座', '大臂', '小臂', '夹爪']
let chart: EChartsType | undefined
let resize: ResizeObserver | undefined
let timer: ReturnType<typeof setTimeout> | undefined

function draw() {
  const end = props.points.at(-1)?.time ?? 0
  const rows = props.live ? props.points.filter((p) => p.time >= end - 30_000) : props.points
  const start = props.live ? end - 30_000 : (rows[0]?.time ?? 0)
  const names = props.type === 'imu' ? ['Roll', 'Pitch'] : labels
  chart?.setOption({
    color: palettes, animation: false,
    grid: { top: 32, bottom: 33, left: 48, right: 24 },
    legend: { left: 6, top: 0, textStyle: { color: '#4a6172' } },
    tooltip: { trigger: 'axis', valueFormatter: (value: unknown) => `${value ?? '—'}°` },
    xAxis: {
      type: 'value',
      min: 0,
      max: props.live ? 30 : Math.max(1, (end - start) / 1000),
      axisLabel: { formatter: '{value}s', color: '#587083' },
      axisLine: { lineStyle: { color: '#a7b9c5' } },
      splitLine: { show: true, lineStyle: { color: '#e1eaf1' } },
    },
    yAxis: {
      type: 'value', scale: true,
      axisLabel: { formatter: '{value}°', color: '#587083' },
      splitLine: { lineStyle: { color: '#e1eaf1' } },
    },
    series: names.map((name, i) => ({
      name,
      type: 'line',
      showSymbol: false,
      sampling: 'lttb',
      lineStyle: { width: 2 },
      data: rows.map((p) => [(p.time - start) / 1000,
        props.type === 'imu' ? (i === 0 ? p.roll : p.pitch) : (p.targets?.[i] ?? null)]),
    })),
  }, { notMerge: true })
}

watch(() => [props.points, props.type], () => {
  if (timer) return
  timer = setTimeout(() => { timer = undefined; draw() }, 100)
})
onMounted(() => {
  if (!node.value) return
  chart = init(node.value)
  resize = new ResizeObserver(() => chart?.resize())
  resize.observe(node.value)
  draw()
})
onBeforeUnmount(() => {
  clearTimeout(timer)
  resize?.disconnect()
  chart?.dispose()
})
</script>

<template>
  <div ref="node" class="chart" role="img" :aria-label="type === 'imu' ? '手腕姿态时间曲线' : '四路关节软件目标曲线'"></div>
</template>
