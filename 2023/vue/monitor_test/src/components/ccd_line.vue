
<template>
    <div>
      <div id="ccd" style="width:100%;height:400px"></div>
    </div>
  </template>

  <script>
  import * as echarts from "echarts";
  import { onMounted,onDeactivated } from "vue";
  import { useStore } from 'vuex';
  export default {
    computed: {

    },
    setup() {

      const store = useStore();
      var x_list = [];
      for(let i=0;i<128;i++){
        x_list[i]=i;
        store.state.ccd_data[i]=0;
      }
      onDeactivated(()=>{
        store.state.myccd=null;
        store.state.ccd_data=null;
        store.state.ccd_data=[];
      })
      onMounted(() => {
        store.state.myccd = echarts.init(document.getElementById('ccd'));
        let option = {
          title: {
            text: '数据',

          },
          tooltip: {},
          legend: {
            data: ['数据']
          },
          xAxis: {
            data: x_list,
            // maxInterval: 60* 1000,
            // type: 'time',

          },
          // dataZoom: [
          //   {
          //     show: true,
          //     type: 'inside',
          //     filterMode: 'none',
          //     startValue: -20,
          //     endValue: 20
          //   },
          //   {
          //     show: true,
          //     type: 'inside',
          //     filterMode: 'none',
          //     startValue: -50,
          //     endValue: 50
          //   }
          // ],
          yAxis: {
            // type: 'value'
          },
          series: [
            {
              name: '数据',
              type: 'line',
              data: store.state.ccd_data,
              showSymbol: false,
              clip: true,
            }
          ]
        };
        store.state.myccd.setOption(option);
      });

    },
    methods: {

    }
  };
  </script>

