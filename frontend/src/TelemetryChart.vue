<script setup lang="ts">
import { onBeforeUnmount, onMounted, ref, watch } from 'vue'
import { init, use, type EChartsType } from 'echarts/core'
import { LineChart } from 'echarts/charts'
import { GridComponent, LegendComponent, TooltipComponent } from 'echarts/components'
import { CanvasRenderer } from 'echarts/renderers'
import type { Point, Pose } from './types'
use([LineChart, GridComponent, LegendComponent, TooltipComponent, CanvasRenderer])
const props = defineProps<{ points: Point[]; type: 'imu' | 'joints'; live?: boolean }>()
const node = ref<HTMLDivElement>()
let chart: EChartsType | undefined, resize: ResizeObserver | undefined, timer: ReturnType<typeof setTimeout> | undefined
function draw() {
  const end = props.points.at(-1)?.time ?? 0
  const rows = props.live ? props.points.filter((p) => p.time >= end - 30_000) : props.points
  const start = props.live ? end - 30_000 : (rows[0]?.time ?? 0)
  const keys = ['base_yaw_deg', 'shoulder_down_deg', 'elbow_down_deg'] as const
  const labels = ['底座', '大臂', '小臂']
  const colors = ['#57d9cb', '#8fa9ff', '#eab675']
  const values = (point: Point, pose: Pose | null, index: number) => props.type === 'imu' ? (index === 0 ? point.roll : point.pitch) : pose?.[keys[index]!] ?? null
  chart?.setOption({
    animation: false,
    grid: { top: 33, bottom: 27, left: 45, right: 18 },
    legend: { left: 0, top: 0, itemWidth: 14, itemHeight: 3, textStyle: { color: '#8fa9b9', fontSize: 10 } },
    tooltip: { trigger: 'axis', backgroundColor: '#142431', borderColor: '#355061', textStyle: { color: '#dce8ef', fontSize: 12 }, valueFormatter: (v: unknown) => typeof v === 'number' ? v.toFixed(2) + '°' : '—' },
    xAxis: { type: 'value', min: 0, max: props.live ? 30 : Math.max(1, (end - start) / 1000), axisLabel: { formatter: '{value}s', color: '#6b8494', fontSize: 10 }, axisLine: { lineStyle: { color: '#28404f' } }, splitLine: { show: false } },
    yAxis: { type: 'value', scale: true, axisLabel: { formatter: '{value}°', color: '#6b8494', fontSize: 10 }, splitLine: { lineStyle: { color: '#1c303f' } } },
    series: (props.type === 'imu' ? ['Roll', 'Pitch'].map((name, i) => ({ name, index: i, requested: false })) : labels.flatMap((name, i) => [{ name: name + ' 输出', index: i, requested: false }, { name: name + ' 请求', index: i, requested: true }])).map((entry) => ({
      name: entry.name, type: 'line', showSymbol: false, connectNulls: false,
      lineStyle: { width: entry.requested ? 1 : 2, type: entry.requested ? 'dashed' : 'solid', color: colors[entry.index], opacity: entry.requested ? 0.5 : 1 },
      itemStyle: { color: colors[entry.index] }, sampling: 'lttb',
      data: rows.map((p) => [(p.time - start) / 1000, values(p, entry.requested ? p.requested : p.output, entry.index)]),
    })),
  }, { notMerge: true })
}
watch(() => [props.points, props.type, props.live], () => {
  if (timer) return
  timer = setTimeout(() => { timer = undefined; draw() }, 100)
})
onMounted(() => {
  if (!node.value) return
  chart = init(node.value)
  resize = new ResizeObserver(() => chart?.resize())
  resize.observe(node.value); draw()
})
onBeforeUnmount(() => { clearTimeout(timer); resize?.disconnect(); chart?.dispose() })
</script>
<template><div ref="node" class="chart" role="img" :aria-label="type === 'imu' ? '手腕姿态时间曲线' : '关节请求和软件输出目标时间曲线'"></div></template>
