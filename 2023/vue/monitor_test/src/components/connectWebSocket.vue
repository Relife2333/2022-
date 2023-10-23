<template>
    <div class="container">
        <div class="mb-3 mt-3">
            <input class="form-control" v-model="ip" placeholder="IP地址">
        </div>
        <div class="mb-3 mt-3">
            <input class="form-control" v-model="port" placeholder="端口号">
        </div>
        <div class="mb-3 mt-3">
            <button type="button" class="btn btn-danger" v-if="isConnected" @click="webclose">断开</button>
            <button type="button" class="btn  btn-success" v-else @click="connectWebSocket">连接</button>
        </div>
        <p v-if="isConnected"><strong>已连接</strong></p>
        <p v-else><strong>未连接</strong></p>
    </div>
</template>
<script>

import { mapActions, mapState } from 'vuex';
export default {
    data() {
        return {
            ip: '192.168.31.145',
            port: '1234',
            isConnected: false,
            web_count: [],
            // receive_message_temp:'',
            time_now: [],
        }
    },
    computed: {
        ...mapState(['receive_message', 'socket']),
    },
    methods: {
        ...mapActions(['updateSocket', 'updateReceive_message']),
        connectWebSocket() {
            if ((this.ip != '') && (this.port != '')) {

                const url = `ws://${this.ip}:${this.port}`;
                this.$store.state.socket = new WebSocket(url);
                // this.updateSocket(this.socket);
                this.socket.onopen = () => {
                    // this.updateSocket(this.socket);
                    console.log("open OK");
                    this.isConnected = true;
                };
                this.socket.onclose = () => {
                    console.log("close OK");
                    this.isConnected = false;
                };
                this.socket.onmessage = (event) => {


                    if (this.$route.path === '/echartsImg') {
                        this.$store.state.receive_message = event.data;
                        let jsonObj = JSON.parse(this.$store.state.receive_message);
                        this.$store.state.web_data.shift();
                        this.$store.state.web_data.push([this.web_count, jsonObj.data]);
                        this.$store.state.myChart.setOption({
                            series: [
                                {
                                    data: this.$store.state.web_data
                                }
                            ]
                        });
                        // console.log(jsonObj.data);
                        if (this.web_count < 19) {
                            this.web_count++;
                        } else {
                            for (let i = 0; i < 20; i++) {
                                this.$store.state.web_data[i][0] -= 1;
                            }
                        }
                    }

                    else if(this.$route.path === '/image_upload') {
                        console.log(this.$route.path);
                        this.$store.state.receive_message = event.data;
                        this.$store.state.imageData = 'data:image/jpeg;base64,' + this.$store.state.receive_message;
                    }

                     else if(this.$route.path === '/ccd_line') {
                        this.$store.state.receive_message = event.data;
                        let jsonObj = JSON.parse(this.$store.state.receive_message);
                        this.$store.state.ccd_data = jsonObj.ccd;
                        this.$store.state.myccd.setOption({
                            series: [
                                {
                                    data: this.$store.state.ccd_data
                                }
                            ]
                        });
                    }






                };
            }
        },
        webclose() {
            this.socket.close();
            this.socket = null;
            console.log('close is ok!')
            this.updateSocket(this.socket);
            this.isConnected = false;
        },
    },


};
</script>
