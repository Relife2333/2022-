import asyncio
import websockets
import cv2
import base64
import numpy as np
import matplotlib.pyplot as plt

# 处理接收到的图像数据
def handle_image_data(image_data):
    # 将Base64编码的字符串转换为图像数据
    image_bytes = base64.b64decode(image_data)
    image_array = np.frombuffer(image_bytes, dtype=np.uint8)
    image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)

    # 显示图像
    cv2.imshow('Camera', image)

# 连接WebSocket服务器
async def connect_server():
    async with websockets.connect('ws://192.168.31.119:1234') as websocket:
        while True:
            image_data = await websocket.recv()
            handle_image_data(image_data)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

# 运行客户端
asyncio.get_event_loop().run_until_complete(connect_server())
