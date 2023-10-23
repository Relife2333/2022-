import { createRouter, createWebHashHistory } from 'vue-router'

const routes = [
  {
  path: '/',
  redirect: '/echartsImg'
},
{
  path: '/echartsImg',
  name: 'echartsImg',
  component: () => import('../components/echartsImg.vue'),
},
{
  path: '/ccd_line',
  name: 'ccd_line',
  component: () => import('../components/ccd_line.vue'),
},
{
  path: '/image_upload',
  name: 'image_upload',
  component: () => import('../components/image_upload.vue'),
}
]

const router = createRouter({
  history: createWebHashHistory(),
  routes
})

export default router
