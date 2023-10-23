#include <Arduino.h>
#include <SimpleFOC.h>
// #include <Wire.h>
// #include "ArduinoJson.h"
#include "Preferences.h"
#include "../lib/pid_kun/pid_kun.h"
// put function declarations here:
pid_kun pid_kunx;
pid_kun pid_kuny;
TaskHandle_t motor_task;
TaskHandle_t Task2;
void motor_code(void *pvParameters);
void Task2code(void *pvParameters);
void setup()
{
  Serial.begin(115200);

  // 创建任务，分配到不同的核心
  xTaskCreatePinnedToCore(
      motor_code,  /* Task function. */
      "motortask", /* name of task. */
      150000,      /* Stack size of task */
      NULL,        /* parameter of the task */
      1,           /* priority of the task */
      &motor_task, /* Task handle to keep track of created task */
      1);          /* pin task to core 0 */

  xTaskCreatePinnedToCore(
      Task2code, /* Task function. */
      "Task2",   /* name of task. */
      10000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      1,         /* priority of the task */
      &Task2,    /* Task handle to keep track of created task */
      0);
}

void loop()
{
  vTaskDelay(1000);
}
TimerHandle_t exampleTimer;
uint8_t motor_mode = 0;
Preferences preferences;
void timerCallback(TimerHandle_t xTimer);
MagneticSensorI2C sensory = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensorx = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motory = BLDCMotor(11);
BLDCDriver3PWM drivery = BLDCDriver3PWM(4, 5, 6, 7);
BLDCMotor motorx = BLDCMotor(11);
BLDCDriver3PWM driverx = BLDCDriver3PWM(1, 2, 42, 41);
Commander command = Commander(Serial);
float x_last_angle_0 = 0, y_last_angle_0 = 0, x_last_angle_1 = 0, y_last_angle_1 = 0,
      x_last_angle_2 = 0, y_last_angle_2 = 0, x_last_angle_3 = 0, y_last_angle_3 = 0,
      x_last_angle_4 = 0, y_last_angle_4 = 0, x_last_angle_5 = 0, y_last_angle_5 = 0,
      x_last_angle_6 = 0, y_last_angle_6 = 0, x_last_angle_7 = 0, y_last_angle_7 = 0,
      x_last_angle_8 = 0, y_last_angle_8 = 0;
float zero_positionx = 0, zero_positiony = 0;
float target_angley = 0, target_anglex = 0, target_angley_temp = 0, target_anglex_temp = 0;
void doTargetx(char *cmd) { command.scalar(&target_anglex_temp, cmd); }
void doTargety(char *cmd) { command.scalar(&target_angley_temp, cmd); }

void doP(char *cmd) { command.scalar(&motorx.P_angle.P, cmd); }
void doI(char *cmd) { command.scalar(&motorx.P_angle.I, cmd); }
void doD(char *cmd) { command.scalar(&motorx.P_angle.D, cmd); }
// void IRAM_ATTR orangedata(void);
void read_key(void);
void motor_code_init(void);
void event_init(void);
void motor_code(void *pvParameters) // This is a task.
{
  motor_code_init();
  event_init();
  while (1)
  {
    switch (motor_mode)
    {
    case 0:
      command.run();
      motory.loopFOC();
      motorx.loopFOC();
      if ((target_anglex_temp != 0) || (target_angley_temp != 0))
      {
        target_anglex = target_anglex + pid_kunx.PID_increment(target_anglex_temp);
        target_angley = target_angley + pid_kuny.PID_increment(target_angley_temp);
      }
      // target_anglex = target_anglex + target_anglex_temp / 7000.0;
      // target_angley = target_angley + target_angley_temp / 7000.0;
      motory.move((target_angley + y_last_angle_0));
      motorx.move((target_anglex + x_last_angle_0));
      // Serial.printf("x:%f,y:%f\n", sensorx.getSensorAngle() + zero_positionx, sensory.getSensorAngle() - zero_positiony);
      // motory.move(zero_positiony);
      // motorx.move(x_last_angle_0);
      // motorx.move(-x_last_angle_0);
      // motorx.move(zero_positionx);
      // Serial.printf("y_L:%f,y:%f\n",y_last_angle_0, sensory.getSensorAngle());
      // Serial.printf("x_L:%f,x:%f,y_L:%f,y:%f\n",x_last_angle_0, sensorx.getSensorAngle(),y_last_angle_0, sensory.getSensorAngle());
      // motorx.move(x_last_angle_0);
      target_anglex_temp = 0;
      target_angley_temp = 0;

      // vTaskDelay(1);
      break;
    case 1:
      xTimerStart(exampleTimer, 0);
      motor_mode = 0;
      break;
    case 2:
      command.run();
      motory.loopFOC();
      motorx.loopFOC();
      motorx.move(target_anglex + x_last_angle_0);
      motory.move(target_angley + y_last_angle_0);
      break;
    default:
      break;
    }
    read_key();
  }
}
void motor_code_init(void)
{
  float p = 000, i = 1 / 4000.0, d = 1 / 1000;
  float integral_max = 0.0037714285714286, integral_min = -0.0037714285714286;
  pid_kunx.set_pid(p, i, d);
  pid_kuny.set_pid(p, i, d);
  pid_kunx.set_output_val_range(integral_max, integral_min);
  pid_kuny.set_output_val_range(integral_max, integral_min);
  preferences.begin("my-app", false); // 打开存储区域
  x_last_angle_0 = preferences.getFloat("x_last_angle_0", 4);
  y_last_angle_0 = preferences.getFloat("y_last_angle_0", -2);
  x_last_angle_1 = preferences.getFloat("x_last_angle_1", 4);
  y_last_angle_1 = preferences.getFloat("y_last_angle_1", -2);
  x_last_angle_2 = preferences.getFloat("x_last_angle_2", 4);
  y_last_angle_2 = preferences.getFloat("y_last_angle_2", -2);
  x_last_angle_3 = preferences.getFloat("x_last_angle_3", 4);
  y_last_angle_3 = preferences.getFloat("y_last_angle_3", -2);
  x_last_angle_4 = preferences.getFloat("x_last_angle_4", 4);
  y_last_angle_4 = preferences.getFloat("y_last_angle_4", -2);
  x_last_angle_5 = preferences.getFloat("x_last_angle_5", 4);
  y_last_angle_5 = preferences.getFloat("y_last_angle_5", -2);
  x_last_angle_6 = preferences.getFloat("x_last_angle_6", 4);
  y_last_angle_6 = preferences.getFloat("y_last_angle_6", -2);
  x_last_angle_7 = preferences.getFloat("x_last_angle_7", 4);
  y_last_angle_7 = preferences.getFloat("y_last_angle_7", -2);
  x_last_angle_8 = preferences.getFloat("x_last_angle_8", 4);
  y_last_angle_8 = preferences.getFloat("y_last_angle_8", -2);
  // 关闭存储区域
  preferences.end();
  Serial1.begin(115200, SERIAL_8N1, 17, 18);
  Wire.setPins(15, 16);
  sensory.init();
  // Serial.println(sensory.getSensorAngle());
  motory.linkSensor(&sensory);
  drivery.voltage_power_supply = 12;
  drivery.init();
  motory.linkDriver(&drivery);
  motory.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motory.controller = MotionControlType::angle;
  // motory.PID_velocity.P = 0.2f;
  // motory.PID_velocity.I = 15;
  // motory.PID_velocity.D = 0.001;
  motory.voltage_limit = 6;
  motory.LPF_velocity.Tf = 0.05f;
  motory.P_angle.P = 25;
  motory.P_angle.I = 20;
  motory.P_angle.D = 0.0;
  motory.velocity_limit = 0.15;
  motory.useMonitoring(Serial);
  // drivery.init();
  command.add('X', doTargetx, "targetx angle");
  command.add('Y', doTargety, "targety angle");
  command.add('P', doP, "P");
  command.add('I', doI, "I");
  command.add('D', doD, "D");
  Wire1.setPins(40, 39);
  sensorx.init(&Wire1);
  motorx.linkSensor(&sensorx);
  driverx.voltage_power_supply = 12;
  driverx.init();
  motorx.linkDriver(&driverx);
  motorx.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motorx.controller = MotionControlType::angle;
  // motorx.PID_velocity.P = 0.2f;
  // motorx.PID_velocity.I = 15;
  // motorx.PID_velocity.D = 0.001;
  motorx.voltage_limit = 6;
  motorx.LPF_velocity.Tf = 0.05f;
  motorx.P_angle.P = 25;
  motorx.P_angle.I = 20;
  motorx.P_angle.D = 0;
  motorx.velocity_limit = 0.15;
  // motor.shaft_angle = 0;
  // motorx.shaft_angle = 0;
  // float temp=motorx.P_angle.P;
  motorx.useMonitoring(Serial);
  // while (motorx.P_angle.P==temp)
  // {
  //   command.run();
  //   vTaskDelay(1);
  // }
  // motorx.sensor_direction = 1;
  motorx.init();
  motory.init();
  motorx.initFOC(6.07, CW);
  motory.initFOC(4.01, CW);
  // motorx.initFOC();
  // motory.initFOC();
  // // driverx.init();
  // motorx.sensor_direction = -1;

  Serial.println(motorx.shaft_angle);
  //   Serial.println(motorx.shaft_angle);
  zero_positionx = motorx.shaft_angle;
  zero_positiony = motory.shaft_angle;
  Serial.println(motory.shaft_angle);
  // motorx.shaft_angle = 0;
  // Serial1.onReceive(orangedata);
  Serial.println(x_last_angle_0);
  Serial.println(y_last_angle_0);
  Serial.println(sensorx.getSensorAngle());
  Serial.println(sensory.getSensorAngle());
  // target_anglex = x_last_angle_0;
  // target_angley = y_last_angle_0;
  // Serial1.println("1");
}
void event_init(void)
{
  exampleTimer = xTimerCreate("Example Timer",     // 定时器名称
                              pdMS_TO_TICKS(2000), // 定时器周期，这里设置为1000ms
                              pdTRUE,              // 设置为自动重载模式
                              NULL,                // 可选的标识符参数
                              timerCallback);
  pinMode(8, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(46, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  // xTimerStart(exampleTimer, 0);
  // 启动定时器
}
int tim_i = 0;
void read_key(void)
{
  static uint8_t key1_mode = 0;
  static uint8_t key3_mode = 0;
  if (digitalRead(10) == 0)
  {
    vTaskDelay(20);
    if (digitalRead(10) == 0)
    {
      Serial.println("key4");
      while (digitalRead(10) == 0)
      {
        vTaskDelay(1);
      }
      vTaskDelay(10);
      Serial1.println("1");
    }
  }
  if (digitalRead(9) == 0)
  {
    vTaskDelay(20);
    if (digitalRead(9) == 0)
    {
      Serial.println("key3");
      while (digitalRead(9) == 0)
      {
        command.run();
        motory.loopFOC();
        motorx.loopFOC();
        motorx.move(target_anglex + x_last_angle_0);
        motory.move(target_angley + y_last_angle_0);
      }
      if (motor_mode == 0)
      {
        motor_mode = 2;
      }
      else if (motor_mode == 2)
      {
        motor_mode = 0;
      }
      // vTaskDelay(100);
    }
  }

  if (digitalRead(8) == 0)
  {
    vTaskDelay(20);
    if (digitalRead(8) == 0)
    {
      Serial.println("key0");
      while (digitalRead(8) == 0)
      {
        vTaskDelay(1);
      }
      target_anglex = 0;
      target_angley = 0;
      if (xTimerIsTimerActive(exampleTimer) == pdTRUE)
      {
        tim_i = 0;
        xTimerStop(exampleTimer, 0);
      }
      motor_mode = 2;
      vTaskDelay(100);
    }
  }

  if (digitalRead(3) == 0)
  {
    vTaskDelay(20);
    if (digitalRead(3) == 0)
    {
      Serial.println("key1");
      while (digitalRead(3) == 0)
      {
        switch (key1_mode)
        {
        case 0:
          x_last_angle_0 = sensorx.getSensorAngle();
          y_last_angle_0 = sensory.getSensorAngle();
          preferences.begin("my-app", false); // 打开存储区域
          preferences.putFloat("x_last_angle_0", x_last_angle_0);
          preferences.putFloat("y_last_angle_0", y_last_angle_0);
          // 关闭存储区域
          preferences.end();
          break;
        case 1:
          x_last_angle_1 = sensorx.getSensorAngle();
          y_last_angle_1 = sensory.getSensorAngle();
          preferences.begin("my-app", false); // 打开存储区域
          preferences.putFloat("x_last_angle_1", x_last_angle_1);
          preferences.putFloat("y_last_angle_1", y_last_angle_1);
          // 关闭存储区域
          preferences.end();
          break;
        case 2:
          x_last_angle_2 = sensorx.getSensorAngle();
          y_last_angle_2 = sensory.getSensorAngle();
          preferences.begin("my-app", false); // 打开存储区域
          preferences.putFloat("x_last_angle_2", x_last_angle_2);
          preferences.putFloat("y_last_angle_2", y_last_angle_2);
          // 关闭存储区域
          preferences.end();
          break;
        case 3:
          x_last_angle_3 = sensorx.getSensorAngle();
          y_last_angle_3 = sensory.getSensorAngle();
          preferences.begin("my-app", false); // 打开存储区域
          preferences.putFloat("x_last_angle_3", x_last_angle_3);
          preferences.putFloat("y_last_angle_3", y_last_angle_3);
          // 关闭存储区域
          preferences.end();
          break;
        case 4:
          x_last_angle_4 = sensorx.getSensorAngle();
          y_last_angle_4 = sensory.getSensorAngle();
          preferences.begin("my-app", false); // 打开存储区域
          preferences.putFloat("x_last_angle_4", x_last_angle_4);
          preferences.putFloat("y_last_angle_4", y_last_angle_4);
          // 关闭存储区域
          preferences.end();
          break;
        case 5:
          x_last_angle_5 = sensorx.getSensorAngle();
          y_last_angle_5 = sensory.getSensorAngle();
          preferences.begin("my-app", false); // 打开存储区域
          preferences.putFloat("x_last_angle_5", x_last_angle_5);
          preferences.putFloat("y_last_angle_5", y_last_angle_5);
          // 关闭存储区域
          preferences.end();
          break;
        case 6:
          x_last_angle_6 = sensorx.getSensorAngle();
          y_last_angle_6 = sensory.getSensorAngle();
          preferences.begin("my-app", false); // 打开存储区域
          preferences.putFloat("x_last_angle_6", x_last_angle_6);
          preferences.putFloat("y_last_angle_6", y_last_angle_6);
          // 关闭存储区域
          preferences.end();
          break;
        case 7:
          x_last_angle_7 = sensorx.getSensorAngle();
          y_last_angle_7 = sensory.getSensorAngle();
          preferences.begin("my-app", false); // 打开存储区域
          preferences.putFloat("x_last_angle_7", x_last_angle_7);
          preferences.putFloat("y_last_angle_7", y_last_angle_7);
          // 关闭存储区域
          preferences.end();
          break;
        case 8:
          x_last_angle_8 = sensorx.getSensorAngle();
          y_last_angle_8 = sensory.getSensorAngle();
          preferences.begin("my-app", false); // 打开存储区域
          preferences.putFloat("x_last_angle_8", x_last_angle_8);
          preferences.putFloat("y_last_angle_8", y_last_angle_8);
          // 关闭存储区域
          preferences.end();
          break;
        default:
          break;
        }
      }
      vTaskDelay(100);
      key1_mode++;
    }
  }

  if (digitalRead(46) == 0)
  {
    vTaskDelay(20);
    if (digitalRead(46) == 0)
    {
      Serial.println("key2");
      while (digitalRead(46) == 0)
      {
        vTaskDelay(1);
      }
      vTaskDelay(100);
      motor_mode = 1;
      tim_i = 0;
    }
  }
}
void timerCallback(TimerHandle_t xTimer)
{
  switch (tim_i)
  {
  case 0:
    target_anglex = x_last_angle_1 - x_last_angle_0;
    target_angley = y_last_angle_1 - y_last_angle_0;
    break;
  case 1:
    target_anglex = x_last_angle_2 - x_last_angle_0;
    target_angley = y_last_angle_2 - y_last_angle_0;
    break;
  case 2:
    target_anglex = x_last_angle_3 - x_last_angle_0;
    target_angley = y_last_angle_3 - y_last_angle_0;
    break;
  case 3:
    target_anglex = x_last_angle_4 - x_last_angle_0;
    target_angley = y_last_angle_4 - y_last_angle_0;
    break;
  case 4:
    target_anglex = x_last_angle_5 - x_last_angle_0;
    target_angley = y_last_angle_5 - y_last_angle_0;
    break;
  case 5:
    target_anglex = x_last_angle_6 - x_last_angle_0;
    target_angley = y_last_angle_6 - y_last_angle_0;
    break;
  case 6:
    target_anglex = x_last_angle_7 - x_last_angle_0;
    target_angley = y_last_angle_7 - y_last_angle_0;
    break;
  case 7:
    target_anglex = x_last_angle_8 - x_last_angle_0;
    target_angley = y_last_angle_8 - y_last_angle_0;
    break;
  case 8:
    target_anglex = x_last_angle_1 - x_last_angle_0;
    target_angley = y_last_angle_1 - y_last_angle_0;
    break;
  case 9:
    target_anglex = x_last_angle_0 - x_last_angle_0;
    target_angley = y_last_angle_0 - y_last_angle_0;
  default:
    break;
  }
  tim_i++;
  if (tim_i == 10)
  {
    tim_i = 0;
    // motory.velocity_limit = 0.02;
    // motorx.velocity_limit = 0.02;
    xTimerStop(exampleTimer, 0);
  }
  // Serial.printf("x:%f,y:%f\n", motorx.shaft_angle-zero_positiony,motory.shaft_angle-zero_position);
}
void IRAM_ATTR orangedata()
{
  // if (Serial1.available())
  // {
  //   // uint8_t c_t  = Serial1.read();
  //   // receivedData += c_t ;
  //   // Serial1.print(c);
  //   // Serial1.println(receivedData);
  //   char receivedData[100];
  //   Serial1.readBytesUntil('/n', receivedData, sizeof(receivedData));
  //   // receivedData[bytesRead] = '/0';
  //   // 使用 deserializeJson() 函数将接收到的 JSON 数据解析到 JsonDocument 对象
  //   // 创建一个 DynamicJsonDocument 对中
  //   // Serial.printf("%s\r\n", receivedData);
  //   DynamicJsonDocument doc(256);
  //   DeserializationError error = deserializeJson(doc, receivedData);
  //   // 检查是否有错误发生
  //   if (error)
  //   {
  //     Serial.print("Deserialization failed: ");
  //     Serial.println(error.c_str());
  //     return;
  //   }

  //   // 从 JsonDocument 对象中提取数据
  //   float x = doc["x"];
  //   float y = doc["y"];
  //   // b = doc["b"];
  //   // c = doc["c"];
  //   // // 打印提取的数据
  //   Serial.print("x: ");
  //   Serial.println(x);
  //   Serial.print("y: ");
  //   Serial.println(y);
  //   // Serial1.print("b: ");
  //   // Serial1.println(b);
  //   // Serial1.print("c: ");
  //   // Serial1.println(c);
  //   // 清空接收数据的字符串，准备接收下一条 JSON 数据
  // }
}
void Task2code(void *pvParameters) // This is a task.
{
  while (1)
  {
    vTaskDelay(1000);
  }
}
