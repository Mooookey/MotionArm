import { createApp } from 'vue'
import { createPinia } from 'pinia'
import { createRouter, createWebHistory } from 'vue-router'
import App from './App.vue'
import Workspace from './Workspace.vue'
import './style.css'

const routes = [
  { path: '/', name: 'dashboard', component: Workspace },
  { path: '/manual', name: 'manual', component: Workspace },
  { path: '/teaching', name: 'teaching', component: Workspace },
  { path: '/replay', name: 'replay', component: Workspace },
  { path: '/settings', name: 'settings', component: Workspace },
  { path: '/:pathMatch(.*)*', redirect: '/' },
]

createApp(App)
  .use(createPinia())
  .use(createRouter({ history: createWebHistory(), routes }))
  .mount('#app')
