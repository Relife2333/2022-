import cv2
import base64
import asyncio
import websockets

# 读取摄像头数据
async def read_camera(websocket, path):
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 将图像转换为Base64编码的字符串
        _, buffer = cv2.imencode('.jpg', frame)
        image_str = base64.b64encode(buffer).decode('utf-8')

        # 发送图像数据到Web前端
        await websocket.send(image_str)
    cap.release()

# 建立WebSocket连接
start_server = websockets.serve(read_camera, '192.168.31.110', 1234)

# 运行WebSocket服务
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
