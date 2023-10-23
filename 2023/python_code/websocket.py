import asyncio
import json
import websockets
import random
async def send_json(websocket, path):
    number = 0
    while True:
        # 创建一个JSON数据
        data = {
            "ccd": number,
        }

        # 将JSON数据转换为字符串
        json_data = json.dumps(data)
        number = [random.randint(1, 100) for _ in range(128)]
        # 发送JSON数据给客户端
        await websocket.send(json_data)
        # 等待1秒钟

        await asyncio.sleep(1)

# 启动WebSocket服务器
start_server = websockets.serve(send_json, '192.168.31.145', 1234)

# 运行事件循环
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
