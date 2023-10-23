import cv2 as cv
import numpy as np
import serial
import json
import multiprocessing



x = []
y = []
z = []
s = []

def sender(receiver):
    while True:
        if receiver.poll():
            a = receiver.recv()
            json_str = json.dumps(a)+'/r/n'
            if ser.out_waiting == 0:
                ser.write(json_str.encode())



def zhu(sender):
    data = {
    'a': 0,
    'b': 0,
    'c': 0
}
    cap = cv.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if ret == True:
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            lower_red1 = np.array([0, 30, 30])# 第一个区间的下界
            upper_red1 = np.array([15, 255, 255]) # 第一个区间的上界
            lower_red2 = np.array([165, 30, 30])# 第二个区间的下界
            upper_red2 = np.array([180, 255, 255])# 第二个区间的上界
            mask_read = cv.bitwise_or(cv.inRange(hsv, lower_red1, upper_red1), cv.inRange(hsv, lower_red2, upper_red2))
            mask_read = cv.GaussianBlur(mask_read, (5, 5), 0)
            mask_read = cv.morphologyEx(mask_read, cv.MORPH_OPEN, np.ones((3, 3), np.uint8),iterations=5)
            mask_read = cv.morphologyEx(mask_read, cv.MORPH_CLOSE, np.ones((3, 3), np.uint8),iterations=5)
            mask_read = cv.threshold(mask_read,50,255,cv.THRESH_BINARY)[1]
            can = cv.Canny(mask_read, 50, 150,apertureSize = 3)
            for i in range(0,480):
                if can[i][10] == 255:
                    z.append(i)
            for i in range(0,480):
                if can[i][629] == 255:
                    y.append(i)
            for i in range(0,640):
                if can[469][i] == 255:
                    x.append(i)
            for i in range(0,640):
                if can[10][i] == 255:
                    s.append(i)
            lists = [z, y, x, s]
            non_empty_count = sum(1 for lst in lists if len(lst) > 0)
            if non_empty_count == 2:
                if len(z)>0 and len(y)>0:
                    if len(z)>=2 and len(y)>=2:
                        point_z = (z[0]+z[-1])/2
                        point_y = (y[0]+y[-1])/2
                        cv.circle(frame,(10,round(point_z)),2,(0,255,0),5)
                        cv.circle(frame,(629,round(point_y)),2,(0,255,0),5)
                        data['a'] = (10+629)/2
                        data['b'] = (point_y+point_z)/2
                        data['c'] = 3
                elif len(x)>0 and len(s)>0:
                    if len(x)>=2 and len(s)>=2:
                        point_x = (x[0]+x[-1])/2
                        point_s = (s[0]+s[-1])/2
                        cv.circle(frame,(round(point_s),10),2,(0,255,0),5)
                        cv.circle(frame,(round(point_x),469),2,(0,255,0),5)
                        data['a'] = (point_s+point_x)/2
                        data['b'] = (10+469)/2
                        data['c'] = 2

            elif non_empty_count == 3:
                if len(x) == 0:
                    if len(s) >= 2 and len(z) >= 2 and len(y) >= 2:
                        point_s = (s[0]+s[-1])/2
                        point_z = (z[0]+z[-1])/2
                        point_y = (y[0]+y[-1])/2
                        cv.circle(frame,(round(point_s),10),2,(0,255,0),5)
                        cv.circle(frame,(10,round(point_z)),2,(0,255,0),5)
                        cv.circle(frame,(629,round(point_y)),2,(0,255,0),5)
                        data['a'] = (10+629)/2
                        data['b'] = (point_y+point_z)/2
                        data['c'] = 3
                if len(s) == 0:
                    if len(x) >= 2 and len(z) >= 2 and len(y) >= 2:
                        point_x = (x[0]+x[-1])/2
                        point_z = (z[0]+z[-1])/2
                        point_y = (y[0]+y[-1])/2
                        cv.circle(frame,(10,round(point_z)),2,(0,255,0),5)
                        cv.circle(frame,(629,round(point_y)),2,(0,255,0),5)
                        cv.circle(frame,(round(point_x),469),2,(0,255,0),5)
                        data['a'] = (10+629)/2
                        data['b'] = (point_y+point_z)/2
                        data['c'] = 3
                if len(z) == 0:
                    if len(x) >= 2 and len(s) >= 2 and len(y) >= 2:
                        point_x = (x[0]+x[-1])/2
                        point_s = (s[0]+s[-1])/2
                        point_y = (y[0]+y[-1])/2
                        cv.circle(frame,(round(point_s),10),2,(0,255,0),5)
                        cv.circle(frame,(629,round(point_y)),2,(0,255,0),5)
                        cv.circle(frame,(round(point_x),469),2,(0,255,0),5)
                        data['a'] = (point_s+point_x)/2
                        data['b'] = (10+469)/2
                        data['c'] = 3
                if len(y) == 0:
                    if len(x) >= 2 and len(z) >= 2 and len(s) >= 2:
                        point_x = (x[0]+x[-1])/2
                        point_z = (z[0]+z[-1])/2
                        point_s = (s[0]+s[-1])/2
                        cv.circle(frame,(round(point_s),10),2,(0,255,0),5)
                        cv.circle(frame,(10,round(point_z)),2,(0,255,0),5)
                        cv.circle(frame,(round(point_x),469),2,(0,255,0),5)
                        data['a'] = (point_s+point_x)/2
                        data['b'] = (10+469)/2
                        data['c'] = 3

            elif non_empty_count == 4:
                if len(x) >= 2 and len(s) >= 2 and len(z) >= 2 and len(y) >= 2:
                    point_x = (x[0]+x[-1])/2
                    point_z = (z[0]+z[-1])/2
                    point_s = (s[0]+s[-1])/2
                    point_y = (y[0]+y[-1])/2
                    cv.circle(frame,(round(point_s),10),2,(0,255,0),5)
                    cv.circle(frame,(10,round(point_z)),2,(0,255,0),5)
                    cv.circle(frame,(629,round(point_y)),2,(0,255,0),5)
                    cv.circle(frame,(round(point_x),469),2,(0,255,0),5)
                    data['a'] = (10+639+round(point_s)+round(point_x))/4
                    data['b'] = (10+469+round(point_z)+round(point_y))/4
                    data['c'] = 4

            print(data)
            # cv.imshow("f",frame)
            sender.send(data)

            if cv.waitKey(1) == 27:
                break
            x.clear()
            y.clear()
            s.clear()
            z.clear()
    cap.release()
    cv.destroyAllWindows()


if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyS1', 115200, timeout=0.1)
    parent_conn, child_conn = multiprocessing.Pipe()

    # 创建发送进程和接收进程
    t1 = multiprocessing.Process(target=zhu, args=(parent_conn,))
    t2 = multiprocessing.Process(target=sender, args=(child_conn,))
    t1.start()
    t2.start()


