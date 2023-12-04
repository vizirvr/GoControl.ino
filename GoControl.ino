#include "I2Cdev.h"
#include "MPU6050.h"
#include "Keyboard.h"
#include "VL53L0X.h"
#define TO_DEG 57.2957f
#define TIME_OUT 20

MPU6050 accgyro;
VL53L0X sensor;

float anglex;
long int t1;

void setup() {
  Serial.begin(9600);
  // инициализация датчика
  accgyro.initialize();
  sensor.init();
  //sensor
}
void loop() {
  // акселлерометр
  long int t = millis();
  if ( t1 < t ) {
    int16_t ax, ay, az, gx, gy, gz;
    float accy, gyrox;

    t1 = t + TIME_OUT;
    accgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // преобразование в единицы гравитации при настройках 1G
    accy = ax / 4096.0;
    //  границы от -1G до +1G
    accy = constrain(accy, -1.0, 1.0);
    // получить значение в градусах
    if ( accy >= 0) {
      anglex = 90 - TO_DEG * acos(accy);
    } else {
      anglex = TO_DEG * acos(-ay) - 90;
    }
  }
  //sensor


    if( anglex >= 80 && sensor.readRangeSingleMillimeters() >= 360 )
    {
    Keyboard.press('w'); // имитируем движение вперёд
    }
    else if( anglex >= 80 )
    {
    Keyboard.press('s'); // имитируем движение вперёд
    }
    else {
    anglex <= 45;
    Keyboard.releaseAll();    // останавливаемся
    }

  //считывание

  Serial.println(anglex);
  Serial.println(sensor.readRangeSingleMillimeters());
  delay(1000);
}
