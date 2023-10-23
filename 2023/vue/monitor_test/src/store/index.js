import { createStore } from 'vuex';

export default createStore({
    state: {
        socket: null,
        receive_message: '',
        web_data:[],
        myChart: null,
        myccd:null,
        ccd_data:[],
        imageData: '',
    },
    getters: {
    },
    mutations: {
        setSocket(state, socket) {
            state.socket = socket;
        },
        setReceive_message(state, value) {
            state.receive_message = value;
            // console.log(this.state.receive_message);
        },
    },
    actions: {
        updateSocket(context, socket) {
            context.commit('setSocket', socket);
        },
        updateReceive_message({ commit }, value) {
            commit('setReceive_message', value);
        },

    },
});
