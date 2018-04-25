#include <MapleFreeRTOS900.h>

#include <Wire.h>

constexpr int PinWheelLeftUp PROGMEM     = PB1;
constexpr int PinWheelLeftDown PROGMEM   = PB0;
constexpr int PinWheelRightUp PROGMEM    = PA7;
constexpr int PinWheelRightDown PROGMEM  = PA6;

constexpr int OutTrig PROGMEM  = PA8;
constexpr int InEcho PROGMEM  = PA9;

constexpr int MaxPwm = 65535;
constexpr int GoPwmLeft = MaxPwm / 2;
constexpr int GoPwmRight = MaxPwm / 2;

constexpr TickType_t xDelay1Ms PROGMEM = 1 / portTICK_PERIOD_MS;

enum class WheelsState
{
  RotationRight,
  RotationLeft,
  Stop,
  Go,
  Back
};

enum class Distance
{
  Close,
  Middle,
  Far,
  Free
};

static Distance barrierDistance = Distance::Free;

WheelsState lastState = WheelsState::Stop;

uint16_t MeasureUltraEcho() 
{
  //запустить измерение
  digitalWrite(OutTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(OutTrig, LOW);
  uint16_t duration = pulseIn(InEcho, HIGH, 2400); //считываем длительность времени прохождения эха  
  if (!duration) 
    duration = 2400;
  return duration;
}

uint16_t findSimilar(uint16_t *buf, uint8_t size_buff, uint8_t range) 
{
  uint8_t maxcomp=0;
  uint16_t mcn=0;
  uint16_t comp;
  range++; 

  for (uint8_t i=0; i<size_buff; ++i)
  {
    comp=buf[i]; 
    uint8_t n=0;
    for (uint8_t j=0; j<size_buff; j++) 
    { 
      if (buf[j]>comp-range && buf[j]<comp+range) 
        ++n;
    }
    if (n > maxcomp)
    {
      maxcomp=n;
      mcn=comp;
    }   
  }
 return mcn;
}

uint16_t GetCurrentDistance() 
{
  #define size_buff 5 //размер массива sensor
  uint16_t sensor[size_buff]; //массив для хранения замеров дальномера
  for (uint8_t i = 0; i < size_buff; ++i) //производим несколько замеров
  {
    sensor[i] = MeasureUltraEcho();  //сохранить в массиве
    vTaskDelay(xDelay1Ms);
  }
  auto duration = findSimilar(sensor, size_buff, 58);
  return duration / 58; // переводим в см
}

uint16_t GetFreeDistance()
{
  constexpr uint16_t MaxDelta = 5;
  static bool isFirst = true;
  static uint16_t pred = 0;
  uint16_t cm = GetCurrentDistance();
  if (isFirst)
  {
    pred = cm;
    isFirst = false;
    Serial.println(cm);
    return cm;
  }

  if (pred != cm)
    Serial.println(cm);
  pred = cm;
  return cm;
}

static void vUltraSonicTask(void *pvParameters) 
{
  pinMode(OutTrig, OUTPUT);
  pinMode(InEcho, INPUT);

  digitalWrite(OutTrig, LOW);
  vTaskDelay(xDelay1Ms);

  for (;;)
  {
    auto cm = GetFreeDistance();
    if (cm < 10)
      barrierDistance = Distance::Close;
    else if (cm < 20)
      barrierDistance = Distance::Middle;
    else if (cm < 40)
      barrierDistance = Distance::Far;
    else
      barrierDistance = Distance::Free;
    vTaskDelay(xDelay1Ms);
  }
}

static void vWheelsTask(void *pvParameters) 
{
  static bool rotateLeft = false;
  for (;;)
  {
    switch (barrierDistance)
    {
      case Distance::Close:
        Rotate(rotateLeft, 255);
        break;
      case Distance::Middle:
        Rotate(rotateLeft, 200);
        break;
      case Distance::Far:
        Rotate(rotateLeft, 130);
        break;
      case Distance::Free:
        WheelsGo();
        rotateLeft = !rotateLeft;
        break;
    }
    vTaskDelay(xDelay1Ms);
  }
}

void setup() 
{
  pinMode(PinWheelLeftUp, PWM);
  pinMode(PinWheelLeftDown, PWM);
  pinMode(PinWheelRightUp, PWM);
  pinMode(PinWheelRightDown, PWM);
//  pinMode(PinWheelLeftUp, OUTPUT);
//  pinMode(PinWheelLeftDown, OUTPUT);
//  pinMode(PinWheelRightUp, OUTPUT);
//  pinMode(PinWheelRightDown, OUTPUT);
  WheelsStop();

  Serial.begin(115200);
  delay(100);

  Wire.begin();

  xTaskCreate(vUltraSonicTask,    "T1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
  xTaskCreate(vWheelsTask,        "T2", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
  vTaskStartScheduler();
}

void loop() 
{
}

void Rotate(bool rotateLeft, TickType_t ms)
{
  WheelsBack();
  vTaskDelay(xDelay1Ms * ms);
  if (rotateLeft)
  {
    WheelsRotationLeft();
  } 
  else
  {
    WheelsRotationRight();
  } 
  vTaskDelay(xDelay1Ms * ms);
  WheelsStop();
  vTaskDelay(xDelay1Ms * ms);
}

void WheelsRotationLeft()
{
  if (lastState == WheelsState::RotationLeft)
  {
    lastState = WheelsState::RotationLeft;
    Serial.println("WheelsRotationLeft");
  }
  WheelLeftUpOff();
  WheelLeftDownOn();
  WheelRightUpOn();
  WheelRightDownOff();
}

void WheelsRotationRight()
{
  if (lastState != WheelsState::RotationRight)
  {
    lastState = WheelsState::RotationRight;
    Serial.println("WheelsRotationRight");
  }
  WheelLeftUpOn();
  WheelLeftDownOff();
  WheelRightUpOff();
  WheelRightDownOn();
}

void WheelsStop()
{
  if (lastState != WheelsState::Stop)
  {
    lastState = WheelsState::Stop;
    Serial.println("WheelsStop");
  }
  WheelLeftUpOff();
  WheelLeftDownOff();
  WheelRightUpOff();
  WheelRightDownOff();
}

void WheelsGo()
{
  if (lastState != WheelsState::Go)
  {
    lastState = WheelsState::Go;
    Serial.println("WheelsGo");
  }
  WheelLeftUpOn();
  WheelLeftDownOff();
  WheelRightUpOn();
  WheelRightDownOff();
}

void WheelsBack()
{
  if (lastState != WheelsState::Back)
  {
    lastState = WheelsState::Back;
    Serial.println("WheelsBack");
  }
  WheelLeftUpOff();
  WheelLeftDownOn();
  WheelRightUpOff();
  WheelRightDownOn();
}

void WheelLeftUpOn()
{
  pwmWrite(PinWheelLeftUp, GoPwmLeft);
//  digitalWrite(PinWheelLeftUp, LOW);
}

void WheelLeftUpOff()
{
  pwmWrite(PinWheelLeftUp, MaxPwm);
//  digitalWrite(PinWheelLeftUp, HIGH);
}

void WheelLeftDownOn()
{
  pwmWrite(PinWheelLeftDown, GoPwmLeft);
//  digitalWrite(PinWheelLeftDown, LOW);
}

void WheelLeftDownOff()
{
  pwmWrite(PinWheelLeftDown, MaxPwm);
//  digitalWrite(PinWheelLeftDown, HIGH);
}

void WheelRightUpOn()
{
  pwmWrite(PinWheelRightUp, GoPwmRight);
//  digitalWrite(PinWheelRightUp, LOW);
}

void WheelRightUpOff()
{
  pwmWrite(PinWheelRightUp, MaxPwm);
//  digitalWrite(PinWheelRightUp, HIGH);
}

void WheelRightDownOn()
{
  pwmWrite(PinWheelRightDown, GoPwmRight);
//  digitalWrite(PinWheelRightDown, LOW);
}

void WheelRightDownOff()
{
  pwmWrite(PinWheelRightDown, MaxPwm);
//  digitalWrite(PinWheelRightDown, HIGH);
}

