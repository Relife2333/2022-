import asyncio
import websockets
import cv2
import base64


# 处理客户端连接
async def handle_client(websocket, path):
    cap = cv2.VideoCapture(0)
    while True:
        try:
            ret, frame = cap.read()
            if not ret:
                break
            # 将图像转换为Base64编码的字符串
            _, buffer = cv2.imencode('.jpg', frame)
            image_str = base64.b64encode(buffer).decode('utf-8')
            # 发送图像数据给客户端
            await websocket.send(image_str)
        except websockets.exceptions.ConnectionClosedError:
            # 客户端断开连接
            cap.release()
            break
# 建立WebSocket连接
start_server = websockets.serve(handle_client, '192.168.31.110', 1234)

# 运行WebSocket服务和摄像头读取
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
