#include <Wire.h>
#include <MPU6050.h>
#include <RF24.h>
#include <KalmanFilter.h> // фильтрация значений
#include "my_math.h"

MPU6050 mpu;
RF24 radio(9, 10);  // Пины подключения к RF-модулю
bool isCalibrated = false;
KalmanFilter kalman;


Vector3 accel = {0, 0, 0};
Vector3 velocity = {0, 0, 0};
Vector3 rotation = {0, 0, 0};
Vector3 position = {0, 0, 0};

// Кол-во миллисекунд с последней итераци
float deltaTime = 0;
float lastTime = 0;

void setup() {
    Serial.begin(9600);
    kalman.init(0.01, 0.1);
    
    Wire.begin();
    mpu.initialize();

    radio.begin();
    radio.openWritingPipe("00001");  // Адрес приемника
    
    delay(1000);

    int16_t ax, ay, az, gx, gy, gz;
    
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    Serial.print("Ускорение: ");
    Serial.print(ax); Serial.print("  ");
    Serial.print(ay); Serial.print("  ");
    Serial.print(az); Serial.print("  ");
    
    Serial.print("Угловая скорость: ");
    Serial.print(gx); Serial.print("  ");
    Serial.print(gy); Serial.print("  ");
    Serial.println(gz);
    lastTime = millis();
}

void loop() {
    int16_t ax, ay, az, gx, gy, gz;
    float ax_f, ay_f, az_f, gx_f, gy_f, gz_f;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);


    // Фильтруем полученные значения и переводим в систему СИ
    ax_f = getConvertedAccel(kalman.update(ax));
    ay_f = getConvertedAccel(kalman.update(ay));
    az_f = getConvertedAccel(kalman.update(az));

    gx_f = getConvertedGyro(kalman.update(gx));
    gy_f = getConvertedGyro(kalman.update(gy));
    gz_f = getConvertedGyro(kalman.update(gz));

    deltaTime = millis() - lastTime;
    lastTime = millis();

    if (isResting(ax_f, ay_f, az_f) && !isCalibrated) {
        calibrateMPU();
    }

    if (isResting(ax_f, ay_f, az_f)) {
        // Передача данных через RF передатчик
        float dataToSend[3] = {position.x, position.y, position.z};
        radio.write(&dataToSend, sizeof(dataToSend[0]) * 3);
        
        // Передача данных на экран
        Serial.print("Координаты: ");
        Serial.print(position.x); Serial.print("  ");
        Serial.print(position.y); Serial.print("  ");
        Serial.print(position.z); Serial.print("  ");
    }

    if (isCalibrated) {
        updateMotion(ax_f, ay_f, az_f, gx_f, gy_f, gz_f)
    }
    
    
}

void calibrateMPU() {
    Serial.println("Калибровка MPU-6050...");
    delay(2000); // Подождать 2 секунды, чтобы устройство успокоилось

    CalibrateAccel(15);
    CalibrateGyro(15);

    isCalibrated = true;
    Serial.println("Калибровка окончена");
}

bool isResting(float ax, float ay, float az) {
    float absAx = abs(ax);
    float absAy = abs(ay);
    float absAz = abs(az);

    // Порог определения состояния покоя
    const float restingThreshold = 0.5; 

    // Проверяем, если ускорение меньше порога,
    // то устройство находится в состоянии покоя
    if (absAx + absAy + absAz < restingThreshold) {
        return true;
    } else {
        return false;
    }
}

void updateMotion(float ax, float ay, float az, float gx, float gy, float gz) {
    accel.x += ax * deltaTime / 1000;
    accel.y += ay * deltaTime / 1000;
    accel.z += az * deltaTime / 1000;

    rotation.x += gx * deltaTime / 1000;
    rotation.y += gy * deltaTime / 1000;
    rotation.z += gz * deltaTime / 1000;

    Matrix3 rotationMatrix = createRotationMatrix(rotation.x, rotation.y, rotation.z);

    Vector3 rotatedAccel = rotationMatrix * accel;
    
    // Обновление скорости на основе ускорения
    velocity.x += rotatedAccel.x * deltaTime / 1000;
    velocity.y += rotatedAccel.y * deltaTime / 1000;
    velocity.z += rotatedAccel.z * deltaTime / 1000;

    // Обновление позиции на основе скорости
    position.x += velocity.x * deltaTime / 1000;
    position.y += velocity.y * deltaTime / 1000;
    position.z += velocity.z * deltaTime / 1000;
}

// Конвертирование в систему СИ
// Конвертирование в м/c^2
float getConvertedAccel(int16_t a) {
    float acc_f = a / 32768 * 2;
    return acc_f;
}

// Конвертирование в градусы/c
float getConvertedGyro(int16_t a) {
    float gyr_f = a / 32768 * 250;
    return gyr_f;
}

