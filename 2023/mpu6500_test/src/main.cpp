#include <MPU6500_WE.h>
#include <Arduino.h>
#include "mecanum_kun.h"
#include "motor_kun.h"
#include "pid_kun.h"
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
mecanum_kun mecanum;

float a_t, b;
WebSocketsServer webSocket = WebSocketsServer(1234);
const char *ssid = "Fourier";
const char *password = "12345678";
void car_move(void *parameter);
void mpu6500(void *parameter);
void websocket(void *parameter);
void setup()
{
    Serial.begin(115200);
    mecanum.begin(4, 5, 6, 7, 15, 16, 17, 18);
    xTaskCreatePinnedToCore(car_move, "car_move", 4096, NULL, 1, NULL, 0);
    // xTaskCreatePinnedToCore(mpu6500, "mpu6500", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(websocket, "websocket", 4096, NULL, 1, NULL, 1);
}

void loop()
{

    vTaskDelay(1000);
}

void car_callback(void *parameter)
{
}

void IRAM_ATTR serial1_Event(void)
{

    if (Serial1.available())
    {
        // uint8_t c_t  = Serial1.read();
        // receivedData += c_t ;
        // Serial1.print(c);
        // Serial1.println(receivedData);
        char receivedData[256];
        int bytesRead=Serial1.readBytesUntil('/n',receivedData,sizeof(receivedData));
        // receivedData[bytesRead]='/0';
        // 使用 deserializeJson() 函数将接收到的 JSON 数据解析到 JsonDocument 对象
        // 创建一个 DynamicJsonDocument 对中
        // Serial1.printf("%s\r\n", receivedData);
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, receivedData);
        // 检查是否有错误发生
        if (error)
        {
            Serial1.print("Deserialization failed: ");
            Serial1.println(error.c_str());
            return;
        }
        // 从 JsonDocument 对象中提取数据
        float at = doc["a"];
        a_t =at;
        // b = doc["b"];
        // c = doc["c"];
        // // 打印提取的数据
        Serial1.print("a: ");
        Serial1.println(a_t);
        // Serial1.print("b: ");
        // Serial1.println(b);
        // Serial1.print("c: ");
        // Serial1.println(c);
        // 清空接收数据的字符串，准备接收下一条 JSON 数据
        
    }
}
void car_move(void *parameter)
{

    pid_kun pid_y(0, 0, 0, 800, -800, 100, -100);
    Serial1.begin(115200, SERIAL_8N1, 8, 3);
    Serial1.onReceive(serial1_Event);
    TimerHandle_t CarTimer = xTimerCreate("car_move_timer", pdMS_TO_TICKS(10), pdTRUE, (void *)1, car_callback);
    xTimerStart(CarTimer, 1);
    while (1)
    {
        // mecanum.go_y(400);
        vTaskDelay(1);
        
        mecanum.go_angle(300,20,);
    }
}

xyzFloat gValue, angle, gyr;
const int csPin = 39; // Chip Select Pin
bool useSPI = true;
MPU6500_WE myMPU6500 = MPU6500_WE(&SPI, csPin, useSPI);
int time_interval = 5;
float position[2];
float yaw;
float v_x, v_y;
int count = 0;
void timerCallback(void *parameter)
{
    // 定时器中断处理代码
    count++;
    // mpu6500加速度积分计算位移
    gValue = myMPU6500.getGValues();
    angle = myMPU6500.getAngles();
    gyr = myMPU6500.getGyrValues();
    yaw += gyr.z * time_interval / 1000;
    v_x += (float)((int)(gValue.x * 100)) / 100 * 9800 * time_interval / 1000;
    v_y += (float)((int)(gValue.y * 100)) / 100 * 9800 * time_interval / 1000;
    position[0] += v_x * time_interval / 1000;
    position[1] += v_y * time_interval / 1000;
}

void mpu6500(void *parameter)
{

    SPI.begin(42, 40, 41, 39);
    if (!myMPU6500.init())
    {
        Serial.println("MPU6500 does not respond");
    }
    else
    {
        Serial.println("MPU6500 is connected");
    }
    Serial.println("Position you MPU6500 flat and don't move it - calibrating...");
    vTaskDelay(100);
    myMPU6500.autoOffsets();
    Serial.println("Done!");
    // myMPU6500.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);
    // myMPU6500.setGyrOffsets(45.0, 145.0, -105.0);
    myMPU6500.enableGyrDLPF();
    // myMPU6500.disableGyrDLPF(MPU6500_BW_WO_DLPF_8800); // bandwdith without DLPF
    myMPU6500.setGyrDLPF(MPU6500_DLPF_0);
    myMPU6500.setSampleRateDivider(5);
    myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
    myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
    myMPU6500.enableAccDLPF(true);
    myMPU6500.setAccDLPF(MPU6500_DLPF_1);
    // esp_timer_create_args_t timerArgs;
    // timerArgs.callback = timerCallback;
    // timerArgs.arg = NULL;
    // esp_timer_handle_t timer;
    // esp_timer_create(&timerArgs, &timer);
    // esp_timer_start_periodic(timer, 1000);
    TimerHandle_t xTimer = xTimerCreate("Timer", pdMS_TO_TICKS(5), pdTRUE, (void *)0, timerCallback);
    xTimerStart(xTimer, 0);
    while (1)
    {
        Serial.println("Acceleration in g (x,y,z):");
        Serial.printf("%.3f %.3f %.3f", gValue.x, gValue.y, gValue.z);
        Serial.println("Gyroscope data in degrees/s: ");
        Serial.print(gyr.x);
        Serial.print("   ");
        Serial.print(gyr.y);
        Serial.print("   ");
        Serial.println(gyr.z);
        Serial.print("Angle x  = ");
        Serial.print(angle.x);
        Serial.print("  |  Angle y  = ");
        Serial.print(angle.y);
        Serial.print("  |  Angle z  = ");
        Serial.println(angle.z);
        Serial.printf("yaw=%.2f x=%.2f y=%.2f \r\n", yaw, v_x, v_y);
        Serial.print("count   = ");
        Serial.println(count);
        count = 0;
        vTaskDelay(1000);
    }
}
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
    switch (type)
    {
    case WStype_DISCONNECTED:
        Serial.printf("[%u] Disconnected!\n", num);
        break;
    case WStype_CONNECTED:
    {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
    }
    break;
    case WStype_TEXT:
        Serial.printf("[%u] Received text: %s\n", num, payload);
        webSocket.sendTXT(num, payload);
        // webSocket.sendTXT(num,"123");
        // Handle received text here
        break;
    case WStype_BIN:
        Serial.printf("[%u] Received binary data of length %u\n", num, length);
        // Handle received binary data here
        break;
    default:
        break;
    }
}
void websocket(void *parameter)
{
    // 任务代码  Serial.begin(115200);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi is OK");

    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    while (1)
    {
        webSocket.loop();
        vTaskDelay(100);
    }
}
