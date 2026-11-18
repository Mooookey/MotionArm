import { createApp } from 'vue'
import { createPinia } from 'pinia'
import { createRouter, createWebHistory } from 'vue-router'
import App from './App.vue'
import './style.css'

const Empty = { render: () => null }
const router = createRouter({ history: createWebHistory(), routes: [
  { path: '/', component: Empty },
  { path: '/recordings', component: Empty },
  { path: '/settings', component: Empty },
  { path: '/manual', redirect: '/' },
  { path: '/teaching', redirect: '/recordings' },
  { path: '/replay', redirect: '/recordings' },
  { path: '/:pathMatch(.*)*', redirect: '/' },
] })
createApp(App).use(createPinia()).use(router).mount('#app')
