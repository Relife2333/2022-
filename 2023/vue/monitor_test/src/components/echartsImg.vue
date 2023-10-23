
<template>
  <div>
    <div id="chart" style="width:100%;height:400px"></div>
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
    for (let i = 0; i < 20; i++) {
      store.state.web_data[i]=[0,0];
    }
    onDeactivated(()=>{
      store.state.myChart=null;
      store.state.web_data=[];
    })
    onMounted(() => {
      store.state.myChart = echarts.init(document.getElementById('chart'));
      let option = {
        title: {
          text: '数据',

        },
        tooltip: {},
        legend: {
          data: ['数据']
        },
        xAxis: {
          // data: time_list,
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
            data: store.state.web_data,
            showSymbol: false,
            clip: true,
          }
        ]
      };
      store.state.myChart.setOption(option);
    });

  },
  methods: {

  }
};
</script>

