import cv2
import numpy as np
import multiprocessing
import serial
import json
import time

def usart(shexiangtou_lan_chlid,shexiangtou_lv_chlid,usart_jiesshou_parent):
    while True:
        if shexiangtou_lan_chlid.poll():
            a = shexiangtou_lan_chlid.recv()
            print(a)
            if ser.out_waiting == 0:
                c = 'X' + str(a[0]) + '\r\n'+ 'Y' + str(a[1]) + '\r\n'
                ser.write(c.encode())
                print(c)

        #接受数据
        # if zuizhong_child.poll():
        #     b = zuizhong_child.recv()
        #     json_str = json.dumps(b)+'/r/n'
        #     if ser.out_waiting == 0:
        #         ser.write(json_str.encode())
        if ser.in_waiting:
            data = ser.readline()
            usart_jiesshou_parent.send(data)
            print(data)

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
        small_rect_points = zuixiao.reshape(4, 2)  # 小矩形的四个角点坐标


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
def zhaojux(shexiangtou_chlid,shexiangtou_lv_parent,usart_jiesshou_chlid):
    rectangles = []
    xuqiu = []
    while True:
        if shexiangtou_chlid.poll():
            frame = shexiangtou_chlid.recv()
            mu = np.zeros((480, 640, 3), np.uint8)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, img = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
            img = cv2.GaussianBlur(img, (3, 3), 0)
            img = cv2.morphologyEx(img, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
            can = cv2.Canny(img, 50, 150)
            # print(c
            contours, hierarchy = cv2.findContours(can, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            # print(len(contours))
            for cnt in contours:
                epsilon = 0.01 * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)

                if len(approx) == 4:
                    area = cv2.contourArea(cnt)

                    if 10000 < area < 90000:
                        # cv2.drawContours(frame, [approx], -1, (0, 255, 0), 3)

                        rectangles.append(approx)
            if len(rectangles) > 0:
                middle_rect_points = zhaozhongxinjuxing(rectangles)
                contours = [np.array(middle_rect_points, dtype=np.int32)]
                # print(contours)
                if not np.array_equal(contours, np.array([])):
                    cv2.drawContours(frame, contours, -1, (0, 255, 0), 1)
                    cv2.drawContours(mu, contours, -1, (255, 255, 255), 1)
                    # gray_image = cv2.Canny(mu, 50, 150)
                    cvt = cv2.cvtColor(mu, cv2.COLOR_BGR2GRAY)
                    xun,_ = cv2.findContours(cvt, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                    cv2.imshow("f",cvt)
                    for cnt in xun:
                        epsilon = 0.01 * cv2.arcLength(cnt, True)
                        approx = cv2.approxPolyDP(cnt, epsilon, True)

                        if len(approx) == 4:
                            area = cv2.contourArea(cnt)
                            # print("good")
                            if 10000 < area < 90000:
                                # 获取边界点坐标并按顺时针方式排序
                                # print(len(xuqiu))
                                if len(xuqiu) == 0:
                                    xuqiu=[cnt,area]
                                    # print("like")
                                if len(xuqiu) != 0:
                                    if area > xuqiu[1]:
                                        # print("hhhh")
                                        xuqiu=[cnt,area]
                                # print(xuqiu)
                    if usart_jiesshou_chlid.poll():
                        data = usart_jiesshou_chlid.recv()
                        print(data)
                        print(len(xuqiu))
                        if data == b'1\r\n' and len(xuqiu) != 0:
                            shexiangtou_lv_parent.send(xuqiu[0])
                            print(xuqiu[0])
                            data = b'0\r\n'
            xuqiu.clear()
            cv2.imshow('frame', frame)
            cv2.imshow('mu', mu)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            rectangles.clear()
def shexiang(shexaingtou_parent,shexiangtou_hong_parent):
    while True:
        cap = cv2.VideoCapture(0)

        while True:
            ret, frame = cap.read()
            if ret:
                shexaingtou_parent.send(frame)
                shexiangtou_hong_parent.send(frame)
    cap.release()



def zhaohongdian(shexiangtou_hong_chlid,shexiangtou_lan_parent,shexiangtou_lv_chlid):
    flag = 0
    i = 1
    x = -1
    y = -1
    w = -1
    h = -1
    lower_red1 = np.array([0, 80, 80])# 第一个区间的下界
    upper_red1 = np.array([15, 255, 255]) # 第一个区间的上界
    lower_red2 = np.array([165, 80, 80])# 第二个区间的下界
    upper_red2 = np.array([180, 255, 255])# 第二个区间的上界
    while True:
        if shexiangtou_hong_chlid.poll():
            frame = shexiangtou_hong_chlid.recv()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask_read = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), cv2.inRange(hsv, lower_red2, upper_red2))
            mask_read = cv2.GaussianBlur(mask_read, (5, 5), 0)
            #椒盐噪声
            mask_read = cv2.medianBlur(mask_read, 5)
            # mask_read = cv2.morphologyEx(mask_read, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8),iterations=3)
            mask_read = cv2.morphologyEx(mask_read, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8),iterations=4)
            mask_read = cv2.morphologyEx(mask_read, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8),iterations=1)
            # mask_read = cv2.erode(mask_read, np.ones((3, 3), np.uint8), iterations=10)
            #膨胀
            mask_read = cv2.dilate(mask_read, np.ones((3, 3), np.uint8), iterations=5)

            mask_read = cv2.threshold(mask_read,50,255,cv2.THRESH_BINARY)[1]
            contours, _ = cv2.findContours(mask_read, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)  # 计算轮廓的面积
                if  100 <area and area <3000 :  # 设定一个阈值，筛选出满足条件的面积较大的白色区域
                    x, y, w, h = cv2.boundingRect(contour)  # 获取轮廓的边界框坐标
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # 在图像上绘制矩形框
            if shexiangtou_lv_chlid.poll() and flag == 0:
                zhi = shexiangtou_lv_chlid.recv()
                print(len(zhi))
                print(zhi)
                flag = 1
                # for xu in zhi:
                #     # 将轮廓转换为具有点坐标的 Python 列表
                #     xu_point = xu.tolist()
                #     flag = 1

                # print(xu_point)
            # if i == 1 and flag == 1:
            #     if abs(((x+x+w)/2)-xu_point[0][0][0])<5 and abs(((y+y+h)/2)-xu_point[0][0][1])<5:
            #         i=0
            if flag == 1 and x != -1 and y != -1 and w != -1 and h != -1:  # 在这里处理每个轮廓的点坐标
                # for poxuint in xu_point:
                #     x, y = poxuint[0]
                point1 = float(zhi[-i][0][0]) - ((x+x+w)/2)
                # point2 = ((y+y+h)/2)-float(xu_point[i][0][1])
                point2 = ((y+y+h)/2) - float(zhi[-i][0][1])
                point3 = (point1,point2)
                # print(point3)
                shexiangtou_lan_parent.send(point3)
                # print(i)
                if abs(((x+x+w)/2)-zhi[-i][0][0])<10 and abs(((y+y+h)/2)-zhi[-i][0][1])<10:
                    i+=1
                if i >= len(zhi):
                    flag = 0
                    i = 1
                    x = -1
                    y = -1
                    w = -1
                    h = -1
            # print(flag)

            cv2.imshow('frame45', mask_read)
            cv2.imshow('fram454', frame)
            cv2.imshow('mask', mask_read)
            if cv2.waitKey(1) & 0xFF == 27:
                break
if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyS1', 115200, timeout=0.01)
    shexaingtou_parent,shexiangtou_chlid = multiprocessing.Pipe()
    shexiangtou_hong_parent,shexiangtou_hong_chlid = multiprocessing.Pipe()
    shexiangtou_lan_parent,shexiangtou_lan_chlid = multiprocessing.Pipe()
    shexiangtou_lv_parent,shexiangtou_lv_chlid = multiprocessing.Pipe()
    usart_jiesshou_parent,usart_jiesshou_chlid = multiprocessing.Pipe()
    t1 = multiprocessing.Process(target=zhaohongdian, args=(shexiangtou_hong_chlid,shexiangtou_lan_parent,shexiangtou_lv_chlid))
    t2 = multiprocessing.Process(target=zhaojux, args=(shexiangtou_chlid,shexiangtou_lv_parent,usart_jiesshou_chlid))
    t3 = multiprocessing.Process(target=usart, args=(shexiangtou_lan_chlid,shexiangtou_lv_chlid,usart_jiesshou_parent))
    t4 = multiprocessing.Process(target=shexiang, args=(shexaingtou_parent,shexiangtou_hong_parent))
    t4.start()
    t1.start()
    t2.start()
    t3.start()
    t4.join()
    t1.join()
    t2.join()
    t3.join()

