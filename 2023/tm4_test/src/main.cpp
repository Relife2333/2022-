#include <Arduino.h>
#include "../lib/mecanum_kun_tm4/mecanum_kun_tm4.h"
// #include "../lib/function_kun_tm4/function_kun_tm4.h"

mecanum_kun_tm4 mecanum;
void setup() {
  mecanum.begin();
}

void loop()
{
  // 执行其他任务
  mecanum.go_y(1000);
}
