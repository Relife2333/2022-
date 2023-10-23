<template>
  <div class="connectWebSocket">
    <div id="connectWebSocket-item">
      <div style="position: relative;top: 10px;">
        <connectWebSocket></connectWebSocket>
      </div>
    </div>
  </div>
  <div>
    <nav style="  display: flex;justify-content: center;">
      <RouterLink to="/echartsImg" style="font-size: 25px; color: #c99266; font-weight: bold;">图表|</RouterLink>
      <RouterLink to="/ccd_line" style="font-size: 25px; color: #c99266; font-weight: bold;">|CCD|</RouterLink>
      <RouterLink to="/image_upload" style="font-size: 25px; color: #c99266; font-weight: bold;">|图传</RouterLink>
    </nav>
    <RouterView></RouterView>
  </div>
</template>
<script>
import connectWebSocket from './components/connectWebSocket.vue'
// import echartsImg from './components/echartsImg.vue';
export default {
  components: {
    'connectWebSocket': connectWebSocket,
    // 'echartsImg': echartsImg,
    // 'get_and_send': get_and_send,
  }
};
</script>
<style>
.connectWebSocket {
  display: flex;
  width: 100%;
  height: 20%;
  justify-content: center;
  background-color: lightgrey;

}
#connectWebSocket-item {
  background-color: cornflowerblue;
  width: 100%;
  height: 22%;
  margin: 10px;
  border-radius: 10px;
}
</style>
