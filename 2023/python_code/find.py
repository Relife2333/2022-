# import cv2
# import numpy as np

# def zhaozhongxinjuxing(approxs):
#     dis = []
#     poin = []
#     zuida = None
#     zuixiao = None
#     if len(approxs) > 1:
#         for approx in approxs:
#             if zuida is None:
#                 zuida = approx
#             elif cv2.contourArea(approx) > cv2.contourArea(zuida):
#                 zuida = approx
#             if zuixiao is None:
#                 zuixiao = approx
#             elif cv2.contourArea(approx) < cv2.contourArea(zuixiao):
#                 zuixiao = approx
#     elif len(approxs) == 1:
#         zuida = approxs
#         zuixiao = approxs

#     if np.array_equal(zuida, zuixiao) == False:
#         # 获取大矩形和小矩形的角点坐标
#         large_rect_points = zuida.reshape(4, 2)  # 大矩形的四个角点坐标
#         small_rect_points = zuixiao.reshape(4, 2)  # 小矩形的四个角点坐标


#         for small_point in small_rect_points:
#             for large_point in large_rect_points:
#                 #计算点的距离
#                 dis.append((np.linalg.norm(small_point - large_point), large_point))
#                 # print(dis)
#                 #dis安装第一个参数从小到大排序
#             dis.sort(key=lambda x:x[0])
#             poin.append(((dis[0][1][0]+small_point[0])/2,(dis[0][1][1]+small_point[1])/2))
#                 # large_rect_points = np.delete(large_rect_points, dis[0][1], axis=0)
#             dis.clear()
#             # print(poin)
#     else:
#         poin = np.array(zuida).reshape(4, 2)




#     return poin



# #创建张黑色的图像
# mu = np.zeros((480, 640, 3), np.uint8)
# cap = cv2.VideoCapture(1)
# rectangles = []

# while True:
#     ret, frame = cap.read()

#     if ret:
#         mu = np.zeros((480, 640, 3), np.uint8)
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         _, img = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
#         img = cv2.GaussianBlur(img, (3, 3), 0)
#         img = cv2.morphologyEx(img, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
#         img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
#         can = cv2.Canny(img, 50, 150)

#         contours, hierarchy = cv2.findContours(can, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

#         for cnt in contours:
#             epsilon = 0.01 * cv2.arcLength(cnt, True)
#             approx = cv2.approxPolyDP(cnt, epsilon, True)

#             if len(approx) == 4:
#                 area = cv2.contourArea(cnt)

#                 if 20000 < area < 45000:
#                     # cv2.drawContours(frame, [approx], -1, (0, 255, 0), 3)

#                     rectangles.append(approx)
#         if len(rectangles) > 0:
#             middle_rect_points = zhaozhongxinjuxing(rectangles)
#             contours = [np.array(middle_rect_points, dtype=np.int32)]
#             print(contours)
#             if not np.array_equal(contours, np.array([])):
#                 cv2.drawContours(mu, contours, -1, (255, 255, 255), 1)
#                 cv2.drawContours(frame, contours, -1, (0, 255, 0), 1)


#         gray_image = cv2.cvtColor(mu, cv2.COLOR_BGR2GRAY)
#         xun = cv2.findContours(gray_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
#         print(xun)
#         cv2.imshow('frame', frame)
#         cv2.imshow('can', can)
#         cv2.imshow('mask', img)
#         cv2.imshow('mu', gray_image)
#         if cv2.waitKey(1) & 0xFF == 27:
#             break
#         rectangles.clear()
# cap.release()
# cv2.destroyAllWindows()

import cv2
import numpy as np
import multiprocessing
import time
import serial
import json
midpoints = []

def chuli(shuju_child,zuizhong_parent,point_child):
    zhong_x = 320
    zhong_y = 240
    x_last = zhong_x
    y_last = zhong_y
    i = 0
    jso = {
    'x': 0,
    'y': 0
    }
    while True:
        if shuju_child.poll():
            data = shuju_child.recv()
            for xu in data:
                # 将轮廓转换为具有点坐标的 Python 列表
                xu_point = xu.tolist()
            # 在这里处理每个轮廓的点坐标
            for poxuint in xu_point:
                x, y = poxuint[0]
                # print(x,y)
                if i == 0:
                    jso["x"] = x - zhong_x
                    jso["y"] = y - zhong_y
                    zuizhong_parent.send(jso)
                    while i==0:
                        if point_child.recv() == b'2\r\n':
                            print("okk")
                            i=1
                    x_last = x
                    y_last = y
                else:
                    jso["x"] = x - x_last
                    jso["y"] = y - y_last
                    zuizhong_parent.send(jso)
                    x_last = x
                    y_last = y
                time.sleep(0.05)
        i = 0
        x_last = zhong_x
        y_last = zhong_y

def zhaozhongxinjuxing(approxs):
    dis = []
    poin = []
    zuida = None
    zuixiao = None
    if len(approxs) > 1:
        for approx in approxs:
            if zuida is None:
                zuida = approx
            elif cv2.contourArea(approx) > cv2.contourArea(zuida):
                zuida = approx
            if zuixiao is None:
                zuixiao = approx
            elif cv2.contourArea(approx) < cv2.contourArea(zuixiao):
                zuixiao = approx
    elif len(approxs) == 1:
        zuida = approxs
        zuixiao = approxs

    if np.array_equal(zuida, zuixiao) == False:
        # 获取大矩形和小矩形的角点坐标
        large_rect_points = zuida.reshape(4, 2)  # 大矩形的四个角点坐标
        small_rect_points = zuixiao.reshape(4, 2)  # 小矩形的四个角点坐标# 小矩形的四个角点坐标

        for i in range(4):
            x = (small_rect_points[i][0] + small_rect_points[(i+1)%4][0]) / 2
            y = (small_rect_points[i][1] + small_rect_points[(i+1)%4][1]) / 2
            midpoints.append((x, y))
        for small_point in small_rect_points:
            for large_point in large_rect_points:
                #计算点的距离
                dis.append((np.linalg.norm(small_point - large_point), large_point))
                # print(dis)
                #dis安装第一个参数从小到大排序
            dis.sort(key=lambda x:x[0])
            poin.append(((dis[0][1][0]+small_point[0])/2,(dis[0][1][1]+small_point[1])/2))
                # large_rect_points = np.delete(large_rect_points, dis[0][1], axis=0)
            dis.clear()
            # print(poin)
    else:
        poin = np.array(zuida).reshape(4, 2)
    return poin

def usart(usart_child,pan_parent,zuizhong_child):
    while True:
        if usart_child.poll():
            a = usart_child.recv()

            if ser.out_waiting == 0:
                ser.write(a.encode())
        #接受数据
        if zuizhong_child.poll():
            b = zuizhong_child.recv()
            json_str = json.dumps(b)+'/r/n'
            if ser.out_waiting == 0:
                ser.write(json_str.encode())
        if ser.in_waiting:
            data = ser.readline()
            pan_parent.send(data)
            print(data)





def juxing(usart_parent,pan_child,shuju_parent,point_parent):
    #创建张黑色的图像
    mu = np.zeros((480, 640, 3), np.uint8)
    cap = cv2.VideoCapture(1)
    rectangles = []

    pan = 0
    jilu = []
    temp1=0
    x_temp=None
    y_temp=None
    while True:
        ret, frame = cap.read()

        if ret:
            if pan_child.poll():
                pan = pan_child.recv()
                # print(pan)

                point_parent.send(pan)
            mu = np.zeros((480, 640, 3), np.uint8)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, img = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
            img = cv2.GaussianBlur(img, (3, 3), 0)
            img = cv2.morphologyEx(img, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
            can = cv2.Canny(img, 50, 150)

            contours, hierarchy = cv2.findContours(can, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                epsilon = 0.01 * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)

                if len(approx) == 4:
                    area = cv2.contourArea(cnt)

                    if 20000 < area < 80000:
                        # cv2.drawContours(frame, [approx], -1, (0, 255, 0), 3)

                        rectangles.append(approx)
            if len(rectangles) > 0:
                middle_rect_points = zhaozhongxinjuxing(rectangles)
                contours = [np.array(middle_rect_points, dtype=np.int32)]
                # print(contours)
                if not np.array_equal(contours, np.array([])):
                    cv2.drawContours(mu, contours, -1, (255, 255, 255), 1)
                    cv2.drawContours(frame, contours, -1, (0, 255, 0), 1)

                    if pan == b'1\r\n':
                        print("ok")
                        gray_image = cv2.cvtColor(mu, cv2.COLOR_BGR2GRAY)
                        gray_image = cv2.Canny(gray_image, 50, 150)
                        xun,_ = cv2.findContours(gray_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                        shuju_parent.send(xun)
                        pan = 0
            if temp1 == 0 and midpoints!=[]:
                for point in midpoints:
                    x, y = point
                    cv2.circle(frame, (int(x), int(y)), 3, (0, 0, 255), -1)  # 在mu图像上绘制中点
                    cv2.putText(frame, f"({x}, {y})", (int(x)+10, int(y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)  # 绘制坐标文字
                    print(f"({x}, {y})")
                    x_temp=int(x)
                    y_temp=int(y)
                temp1 = 1
                midpoints.clear()
            elif  midpoints!=[] and temp1==1 :
                cv2.circle(frame, (int(x_temp), int(y_temp)), 3, (0, 0, 255), -1)  # 在mu图像上绘制中点
                cv2.putText(frame, f"({x_temp}, {y_temp})", (int(x)+10, int(y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                for point in midpoints:
                    x, y = point
                    cv2.circle(frame, (int(x), int(y)), 3, (0, 0, 255), -1)  # 在mu图像上绘制中点
                    cv2.putText(frame, f"({x}, {y})", (int(x)+10, int(y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)  # 绘制坐标文字
                midpoints.clear()
            elif temp1 != 0:
                cv2.circle(frame, (int(x_temp), int(y_temp)), 3, (0, 0, 255), -1)  # 在mu图像上绘制中点
                cv2.putText(frame, f"({x_temp}, {y_temp})", (int(x)+10, int(y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.imshow('frame', frame)
            # cv2.imshow('can', can)
            # cv2.imshow('mask', img)

            if cv2.waitKey(1) & 0xFF == 27:
                break
            rectangles.clear()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    # ser = serial.Serial('/dev/ttyS1', 115200, timeout=0.1)
    usart_parent,usart_child = multiprocessing.Pipe()
    pan_parent,pan_child = multiprocessing.Pipe()
    shuju_parent,shuju_child = multiprocessing.Pipe()
    zuizhong_parent,zuizhong_child = multiprocessing.Pipe()
    point_parent,point_child = multiprocessing.Pipe()
    # t1 = multiprocessing.Process(target=usart,args=(usart_child,pan_parent,zuizhong_child))
    t2 = multiprocessing.Process(target=juxing,args=(usart_parent,pan_child,shuju_parent,point_parent))
    t3 = multiprocessing.Process(target=chuli,args=(shuju_child,zuizhong_parent,point_child))
    # t1.start()
    t3.start()
    t2.start()
    # t1.join()
    t2.join()
    t3.join()
