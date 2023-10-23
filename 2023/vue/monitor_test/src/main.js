import { createApp } from 'vue'
import App from './App.vue'

import store from './store'
import router from './router'
import ElementPlus from 'element-plus'
import '../node_modules/element-plus/dist/index.css'
import '../node_modules/bootstrap/dist/css/bootstrap.css'
import '../node_modules/bootstrap/dist/js/bootstrap.js'

createApp(App).use(router).use(store).use(ElementPlus).mount('#app')
