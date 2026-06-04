/**
 * GY-86 Инклинометр v9.1 (10-осевой + ИСТИНА + UART вывод)
 * Концепция:
 * - Прибор = идеальный куб
 * - tiltX всегда = наклон ВПРАВО
 * - tiltY всегда = наклон ВПЕРЁД (от себя)
 * - Добавлено прерывание на обработку нажатия кнопки D2
 * - В режиме XY: tiltX = вращение, tiltY = перемещение по вертикали
 * - Вывод данных в последовательный порт
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ============================================================================
// Адреса и регистры
// ============================================================================
#define MPU_ADDR      0x68
#define MAG_ADDR      0x1E
#define BARO_ADDR_0   0x76
#define BARO_ADDR_1   0x77

#define MPU_PWR_MGMT_1    0x6B
#define MPU_ACCEL_OUT     0x3B
#define MPU_GYRO_OUT      0x43
#define MPU_WHO_AM_I      0x75
#define MPU_ACCEL_CONFIG  0x1C
#define MPU_GYRO_CONFIG   0x1B

#define BARO_RESET        0x1E
#define BARO_PROM_READ    0xA0
#define BARO_CONV_D1      0x40
#define BARO_CONV_D2      0x50
#define BARO_ADC          0x00

#define MAG_CONFIG_A      0x00
#define MAG_CONFIG_B      0x01
#define MAG_MODE          0x02
#define MAG_DATA_X        0x03
#define MAG_WHO_AM_I      0x0A

// ============================================================================
// Настройки
// ============================================================================
#define BUTTON_PIN        2           // D2 (INT0) - поддерживает внешние прерывания
#define BUTTON_INT        0           // Номер прерывания для пина D2
#define FILTER_ALPHA      4
#define TILT_THRESHOLD    50
#define CALIB_DURATION    5000
#define DEBOUNCE_DELAY    30
#define MULTI_PRESS_TIMEOUT 400
#define LONG_PRESS_TIME   1500
#define AVG_SAMPLES       10
#define AXIS_HYSTERESIS   800
#define AXIS_CONFIRM_MS   300
#define STABLE_THRESHOLD  5
#define STABLE_VARIANCE   30
#define UART_UPDATE_INTERVAL 200      // Интервал обновления UART вывода (увеличен для плавности)
#define SENSOR_READ_INTERVAL 20       // Интервал опроса датчиков (мс)

// ============================================================================
// ФИЛЬТРАЦИЯ - МЕДИАННЫЙ ФИЛЬТР
// ============================================================================
#define MEDIAN_SAMPLES 5

int16_t median5(int16_t* arr) {
    int16_t sorted[MEDIAN_SAMPLES];
    for (uint8_t i = 0; i < MEDIAN_SAMPLES; i++) {
        sorted[i] = arr[i];
        for (int8_t j = i; j > 0 && sorted[j] < sorted[j-1]; j--) {
            int16_t tmp = sorted[j];
            sorted[j] = sorted[j-1];
            sorted[j-1] = tmp;
        }
    }
    return sorted[MEDIAN_SAMPLES / 2];
}

int16_t medBufX[MEDIAN_SAMPLES], medBufY[MEDIAN_SAMPLES], medBufZ[MEDIAN_SAMPLES];
uint8_t medIndex = 0;
bool medReady = false;

// ============================================================================
// Глобальные переменные
// ============================================================================
bool useDegrees = false, calibrating = false;
unsigned long lastOutputTime = 0;
unsigned long lastSensorReadTime = 0;
uint8_t pressCount = 0;

// Переменные кнопки (volatile - используются в прерывании)
volatile bool buttonPressed = false;     // Флаг нажатия для основного цикла
volatile unsigned long buttonDownTime = 0;
volatile bool longPressHandled = false;
volatile uint8_t buttonPressCount = 0;
volatile unsigned long lastButtonPressTime = 0;
int16_t refTiltX = 0, refTiltY = 0;
bool hasCalibration = false;
int16_t lastTiltX = 32767, lastTiltY = 32767;
int16_t expX = 0, expY = 0, expZ = 0;
bool filterInit = false;
int16_t avgTiltX[AVG_SAMPLES], avgTiltY[AVG_SAMPLES];
uint8_t avgIndex = 0;
bool avgReady = false;
int16_t magX = 0, magY = 0, magZ = 0;
bool magAvailable = false;
uint8_t baroI2CAddr = 0;
uint16_t C[7];
bool baroCalibrated = false;
int32_t lastBaroP = 0, lastBaroT = 0;
int32_t basePressure = 101325;
uint8_t verticalAxis = 2, prevVerticalAxis = 2;
int8_t verticalAxisSign = 1;
unsigned long axisChangeTime = 0;
uint8_t stableCount = 0;
bool isStable = false;
int16_t lastStableX = 32767, lastStableY = 32767;
int16_t gyroX = 0, gyroY = 0, gyroZ = 0;
uint8_t displayMode = 0;  // 0 = нормальный, 1 = упрощенный

// ============================================================================
// Вспомогательные функции
// ============================================================================
int16_t expFilter(int16_t prev, int16_t newVal, int16_t alpha) {
    return prev + (newVal - prev) / alpha;
}

void addTiltSample(int16_t x, int16_t y) {
    avgTiltX[avgIndex] = x;
    avgTiltY[avgIndex] = y;
    avgIndex = (avgIndex + 1) % AVG_SAMPLES;
}

void getRobustAvgTilt(int16_t* ax, int16_t* ay) {
    if (!avgReady) {
        *ax = avgTiltX[avgIndex > 0 ? avgIndex - 1 : AVG_SAMPLES - 1];
        *ay = avgTiltY[avgIndex > 0 ? avgIndex - 1 : AVG_SAMPLES - 1];
        return;
    }
    
    int16_t minX = 32767, maxX = -32768;
    int16_t minY = 32767, maxY = -32768;
    int32_t sumX = 0, sumY = 0;
    
    for (uint8_t i = 0; i < AVG_SAMPLES; i++) {
        if (avgTiltX[i] < minX) minX = avgTiltX[i];
        if (avgTiltX[i] > maxX) maxX = avgTiltX[i];
        if (avgTiltY[i] < minY) minY = avgTiltY[i];
        if (avgTiltY[i] > maxY) maxY = avgTiltY[i];
        sumX += avgTiltX[i];
        sumY += avgTiltY[i];
    }
    
    *ax = (int16_t)((sumX - minX - maxX) / (AVG_SAMPLES - 2));
    *ay = (int16_t)((sumY - minY - maxY) / (AVG_SAMPLES - 2));
}

// ============================================================================
// МАТЕМАТИКА УГЛОВ
// ============================================================================
// Для режима XY (X или Y вертикальны):
//   tiltX = вращение (наклон влево/вправо)
//   tiltY = перемещение по вертикали (наклон вперёд/назад)
//
// Для режима Z (Z вертикальна):
//   tiltX = наклон влево/вправо
//   tiltY = наклон вперёд/назад
// ============================================================================
void calcTilt(int16_t ax, int16_t ay, int16_t az,
              int16_t* tiltX, int16_t* tiltY, 
              uint8_t vAxis, int8_t vAxisSign) {
    
    const float RAD_TO_DEG100 = 5729.57795;
    
    switch (vAxis) {
        case 0:  // Физический X вертикален
            {
                // Горизонтальная плоскость: Y-Z
                // tiltX = вращение = наклон влево/вправо = физический Y
                // tiltY = перемещение = наклон вперёд/назад = физический Z
                
                float h_tiltX = (vAxisSign > 0) ? ay : -ay;   // вращение
                float h_tiltY = az;   // перемещение
                
                *tiltX = (int16_t)round(atan2(h_tiltX, sqrt((float)ax*ax + (float)az*az)) * RAD_TO_DEG100);
                *tiltY = (int16_t)round(atan2(h_tiltY, sqrt((float)ax*ax + (float)ay*ay)) * RAD_TO_DEG100);
            }
            break;
            
        case 1:  // Физический Y вертикален
            {
                // Горизонтальная плоскость: X-Z
                // tiltX = вращение = наклон влево/вправо = физический X
                // tiltY = перемещение = наклон вперёд/назад = физический Z
                
                float h_tiltX = (vAxisSign > 0) ? -ax : ax;   // вращение = физический X
                float h_tiltY = (vAxisSign > 0) ? az : az;   // перемещение = физический Z
                
                *tiltX = (int16_t)round(atan2(h_tiltX, sqrt((float)ay*ay + (float)az*az)) * RAD_TO_DEG100);
                *tiltY = (int16_t)round(atan2(h_tiltY, sqrt((float)ax*ax + (float)ay*ay)) * RAD_TO_DEG100);
            }
            break;
            
        case 2:  // Физический Z вертикален
        default:
            {
                // Горизонтальная плоскость: X-Y
                // tiltX = наклон влево/вправо = физический Y (ИНВЕРТИРОВАН!)
                // tiltY = наклон вперёд/назад = физический X
                float h_tiltX = (vAxisSign > 0) ? -ay : ay;   // инвертирован!
                float h_tiltY = (vAxisSign > 0) ? ax : -ax;
                *tiltX = (int16_t)round(atan2(h_tiltX, sqrt((float)ax*ax + (float)az*az)) * RAD_TO_DEG100);
                *tiltY = (int16_t)round(atan2(h_tiltY, sqrt((float)ay*ay + (float)az*az)) * RAD_TO_DEG100);
            }
            break;
    }
}

uint8_t detectVerticalAxis(int16_t ax, int16_t ay, int16_t az, int8_t* vAxisSign) {
    int16_t absX = abs(ax);
    int16_t absY = abs(ay);
    int16_t absZ = abs(az);
    
    const int16_t threshold = 14000;
    const int16_t hyst = AXIS_HYSTERESIS;
    uint8_t candidate = prevVerticalAxis;
    
    if (absX > absY + hyst && absX > absZ + hyst) {
        if (absX > threshold) candidate = 0;
    } else if (absY > absX + hyst && absY > absZ + hyst) {
        if (absY > threshold) candidate = 1;
    } else {
        if (absZ > threshold) candidate = 2;
    }
    
    switch (candidate) {
        case 0: *vAxisSign = (ax >= 0) ? 1 : -1; break;
        case 1: *vAxisSign = (ay >= 0) ? 1 : -1; break;
        case 2: *vAxisSign = (az >= 0) ? 1 : -1; break;
    }
    
    if (candidate != prevVerticalAxis) {
        if (millis() - axisChangeTime > AXIS_CONFIRM_MS) {
            prevVerticalAxis = candidate;
            axisChangeTime = millis();
            return candidate;
        } else {
            return prevVerticalAxis;
        }
    }
    axisChangeTime = millis();
    return prevVerticalAxis;
}

// ============================================================================
// I2C Функции
// ============================================================================

bool deviceExists(uint8_t addr) {
    Wire.beginTransmission(addr);
    return (Wire.endTransmission() == 0);
}

// ============================================================================
// Детектор стабильности
// ============================================================================

void checkStability(int16_t tX, int16_t tY) {
    int16_t diffX = abs(tX - lastStableX);
    int16_t diffY = abs(tY - lastStableY);
    if (diffX < STABLE_VARIANCE && diffY < STABLE_VARIANCE) {
        stableCount++;
        if (stableCount >= STABLE_THRESHOLD) isStable = true;
    } else {
        stableCount = 0;
        isStable = false;
    }
    lastStableX = tX;
    lastStableY = tY;
}

void resetStability() {
    stableCount = 0;
    isStable = false;
}

// ============================================================================
// Инициализация сенсоров
// ============================================================================

bool initMPU6050() {
    Serial.println(F("=== MPU6050 (Accel+Gyro) ==="));
    if (!deviceExists(MPU_ADDR)) {
        Serial.println(F("Not found!"));
        return false;
    }
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1, (uint8_t)true);
    uint8_t whoAmI = Wire.read();
    Serial.print(F("WHO_AM_I: 0x"));
    Serial.println(whoAmI, HEX);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(100);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x37);
    Wire.write(0x02);
    Wire.endTransmission();
    Serial.println(F("I2C_BYPASS: Enabled for HMC5883L"));
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_ACCEL_CONFIG);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_GYRO_CONFIG);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(100);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_ACCEL_OUT);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);
    if (Wire.available() >= 14) {
        expX = (Wire.read() << 8) | Wire.read();
        expY = (Wire.read() << 8) | Wire.read();
        expZ = (Wire.read() << 8) | Wire.read();
        Wire.read(); Wire.read();
        gyroX = (Wire.read() << 8) | Wire.read();
        gyroY = (Wire.read() << 8) | Wire.read();
        gyroZ = (Wire.read() << 8) | Wire.read();
        filterInit = true;
    }
    Serial.print(F("Accel: X=")); Serial.print(expX);
    Serial.print(F(" Y=")); Serial.print(expY);
    Serial.print(F(" Z=")); Serial.println(expZ);
    Serial.print(F("Gyro: X=")); Serial.print(gyroX);
    Serial.print(F(" Y=")); Serial.print(gyroY);
    Serial.print(F(" Z=")); Serial.println(gyroZ);
    Serial.println(F("OK"));
    return true;
}

bool initMS5611() {
    Serial.println(F("=== MS5611 (Barometer) ==="));
    for (uint8_t addr = 0x76; addr <= 0x77; addr++) {
        if (deviceExists(addr)) {
            baroI2CAddr = addr;
            break;
        }
    }
    if (baroI2CAddr == 0) {
        Serial.println(F("Not found at 0x76 or 0x77"));
        return false;
    }
    Serial.print(F("Addr: 0x"));
    Serial.println(baroI2CAddr, HEX);
    Wire.beginTransmission(baroI2CAddr);
    Wire.write(BARO_RESET);
    Wire.endTransmission();
    delay(10);
    Serial.print(F("C1-C6: "));
    for (uint8_t i = 1; i <= 6; i++) {
        Wire.beginTransmission(baroI2CAddr);
        Wire.write(BARO_PROM_READ + (i * 2));
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)baroI2CAddr, (uint8_t)2, (uint8_t)true);
        if (Wire.available() >= 2) {
            C[i] = ((uint16_t)Wire.read() << 8) | Wire.read();
            Serial.print(C[i]);
            Serial.print(F(" "));
        }
    }
    Serial.println();
    if (C[1] == 0 || C[1] == 0xFFFF) {
        Serial.println(F("Invalid PROM coefficients!"));
        return false;
    }
    Serial.println(F("Testing conversion..."));
    Wire.beginTransmission(baroI2CAddr);
    Wire.write(BARO_CONV_D2 + 0x06);
    Wire.endTransmission();
    delay(10);
    Wire.beginTransmission(baroI2CAddr);
    Wire.write(BARO_ADC);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)baroI2CAddr, (uint8_t)3, (uint8_t)true);
    uint32_t D2 = 0;
    if (Wire.available() >= 3) {
        D2 = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
    }
    Wire.beginTransmission(baroI2CAddr);
    Wire.write(BARO_CONV_D1 + 0x06);
    Wire.endTransmission();
    delay(10);
    Wire.beginTransmission(baroI2CAddr);
    Wire.write(BARO_ADC);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)baroI2CAddr, (uint8_t)3, (uint8_t)true);
    uint32_t D1 = 0;
    if (Wire.available() >= 3) {
        D1 = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
    }
    Serial.print(F("D1: ")); Serial.print(D1);
    Serial.print(F(" D2: ")); Serial.println(D2);
    int64_t dT = (int64_t)D2 - ((int64_t)C[5] << 8);
    int64_t TEMP = 2000 + ((dT * (int64_t)C[6]) >> 23);
    Serial.print(F("Temp: "));
    Serial.print((int32_t)(TEMP / 100));
    Serial.print(F("."));
    Serial.println(abs((int32_t)(TEMP % 100)));
    baroCalibrated = true;
    Serial.println(F("OK"));
    return true;
}

bool initHMC5883L() {
    Serial.println(F("=== HMC5883L (Magnetometer) ==="));
    if (!deviceExists(MAG_ADDR)) {
        Serial.println(F("Not found at 0x1E"));
        return false;
    }
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(MAG_WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MAG_ADDR, (uint8_t)3, (uint8_t)true);
    char id[4] = {0};
    if (Wire.available() >= 3) {
        id[0] = Wire.read();
        id[1] = Wire.read();
        id[2] = Wire.read();
    }
    Serial.print(F("ID: ")); Serial.println(id);
    if (id[0] != 'H' || id[1] != '4' || id[2] != '3') {
        Serial.println(F("WARNING: Wrong ID! Expected H43"));
    }
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(MAG_CONFIG_A);
    Wire.write(0x70);
    Wire.write(0x20);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(50);
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(MAG_DATA_X);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MAG_ADDR, (uint8_t)6, (uint8_t)true);
    if (Wire.available() >= 6) {
        int16_t x = (Wire.read() << 8) | Wire.read();
        int16_t z = (Wire.read() << 8) | Wire.read();
        int16_t y = (Wire.read() << 8) | Wire.read();
        Serial.print(F("Mag: X=")); Serial.print(x);
        Serial.print(F(" Y=")); Serial.print(y);
        Serial.print(F(" Z=")); Serial.println(z);
    }
    magAvailable = true;
    Serial.println(F("OK"));
    return true;
}

// ============================================================================
// Чтение данных
// ============================================================================

void readMPU6050Filtered(int16_t* ax, int16_t* ay, int16_t* az) {
    int16_t rawX, rawY, rawZ;
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_ACCEL_OUT);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6, (uint8_t)true);
    if (Wire.available() >= 6) {
        rawX = (Wire.read() << 8) | Wire.read();
        rawY = (Wire.read() << 8) | Wire.read();
        rawZ = (Wire.read() << 8) | Wire.read();
    } else {
        rawX = rawY = rawZ = 0;
    }
    
    medBufX[medIndex] = rawX;
    medBufY[medIndex] = rawY;
    medBufZ[medIndex] = rawZ;
    medIndex = (medIndex + 1) % MEDIAN_SAMPLES;
    if (!medReady && medIndex == 0) medReady = true;
    
    if (medReady) {
        *ax = median5(medBufX);
        *ay = median5(medBufY);
        *az = median5(medBufZ);
    } else {
        *ax = rawX;
        *ay = rawY;
        *az = rawZ;
    }
}

void readMPU6050Gyro(int16_t* gx, int16_t* gy, int16_t* gz) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_GYRO_OUT);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6, (uint8_t)true);
    if (Wire.available() >= 6) {
        *gx = (Wire.read() << 8) | Wire.read();
        *gy = (Wire.read() << 8) | Wire.read();
        *gz = (Wire.read() << 8) | Wire.read();
    }
}

void readHMC5883L() {
    if (!magAvailable) return;
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(MAG_DATA_X);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MAG_ADDR, (uint8_t)6, (uint8_t)true);
    if (Wire.available() >= 6) {
        magX = (Wire.read() << 8) | Wire.read();
        magZ = (Wire.read() << 8) | Wire.read();
        magY = (Wire.read() << 8) | Wire.read();
    }
}

void readMS5611() {
    if (!baroCalibrated) return;
    uint32_t D1, D2;
    Wire.beginTransmission(baroI2CAddr);
    Wire.write(BARO_CONV_D1 + 0x06);
    Wire.endTransmission();
    delay(10);
    Wire.beginTransmission(baroI2CAddr);
    Wire.write(BARO_ADC);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)baroI2CAddr, (uint8_t)3, (uint8_t)true);
    if (Wire.available() >= 3) {
        D1 = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
    }
    Wire.beginTransmission(baroI2CAddr);
    Wire.write(BARO_CONV_D2 + 0x06);
    Wire.endTransmission();
    delay(10);
    Wire.beginTransmission(baroI2CAddr);
    Wire.write(BARO_ADC);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)baroI2CAddr, (uint8_t)3, (uint8_t)true);
    if (Wire.available() >= 3) {
        D2 = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
    }
    int64_t dT = (int64_t)D2 - ((int64_t)C[5] << 8);
    int64_t TEMP = 2000 + ((dT * (int64_t)C[6]) >> 23);
    int64_t TEMP_orig = TEMP;
    int64_t OFF = ((int64_t)C[2] << 16) + (((int64_t)C[4] * dT) >> 7);
    int64_t SENS = ((int64_t)C[1] << 15) + (((int64_t)C[3] * dT) >> 8);
    if (TEMP < 2000) {
        int64_t T2 = (dT * dT) >> 31;
        int64_t delta = TEMP_orig - 2000;
        int64_t OFF2 = (5 * delta * delta) >> 1;
        int64_t SENS2 = (5 * delta * delta) >> 2;
        TEMP = TEMP_orig - T2;
        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }
    int64_t P = (((int64_t)D1 * SENS) >> 21) - OFF;
    P = P >> 15;
    lastBaroP = (int32_t)P * 100;
    lastBaroT = (int32_t)TEMP;
}

// ============================================================================
// Обработка кнопки (аппаратное прерывание)
// ============================================================================
// Обработчик прерывания - вызывается при изменении состояния кнопки (падающий/растущий фронт)
// Определяет: короткое нажатие, долгое нажатие, серийные нажатия (2, 3, 5)
// ============================================================================

void buttonISR() {
    static bool lastState = HIGH;
    bool currentState = digitalRead(BUTTON_PIN);
    unsigned long now = millis();
    
    // Защита от дребезга
    if (now - buttonDownTime < DEBOUNCE_DELAY) return;
    
    // Падающий фронт (кнопка нажата)
    if (currentState == LOW && lastState == HIGH) {
        buttonDownTime = now;
        longPressHandled = false;
    }
    
    // Растущий фронт (кнопка отпущена)
    if (currentState == HIGH && lastState == LOW) {
        unsigned long duration = now - buttonDownTime;
        
        if (!longPressHandled && duration >= DEBOUNCE_DELAY) {
            // Подсчёт серийных нажатий
            if (now - lastButtonPressTime < MULTI_PRESS_TIMEOUT) {
                buttonPressCount++;
            } else {
                buttonPressCount = 1;
            }
            lastButtonPressTime = now;
            
            // Установка флага для основного цикла
            buttonPressed = true;
        }
        
        // Долгое нажатие обрабатываем сразу
        if (duration >= LONG_PRESS_TIME && !longPressHandled) {
            longPressHandled = true;
            buttonPressCount = 4;  // Код долгого нажатия
            buttonPressed = true;
        }
    }
    
    lastState = currentState;
}

// ============================================================================
// Обработка флага кнопки в основном цикле
// ============================================================================
// Считывает флаг, установленный в прерывании, и возвращает код нажатия
// ============================================================================

uint8_t readButton() {
    if (buttonPressed) {
        buttonPressed = false;  // Сброс флага
        uint8_t result = buttonPressCount;
        buttonPressCount = 0;   // Сброс счётчика
        return result;
    }
    return 0;
}

// ============================================================================
// Калибровка нуля
// ============================================================================
// Назначение: установка текущего положения устройства за "ноль"
// Принцип: усреднение углов наклона за CALIB_DURATION (5 сек)
// Применимость: вычитается из текущих углов в основном цикле (tX -= refTiltX)
// Примечание: калибровка глобальная, сохраняется до перезагрузки
// ============================================================================

void runCalibration() {
    calibrating = true;
    Serial.println(F("\n=== CALIBRATION ==="));
    Serial.println(F("Keep device STILL..."));
    int32_t sumTX = 0, sumTY = 0;
    uint16_t samples = 0;
    unsigned long startTime = millis();
    
    // Сбор данных калибровки
    while (millis() - startTime < CALIB_DURATION) {
        int16_t ax, ay, az;
        readMPU6050Filtered(&ax, &ay, &az);
        
        // Экспоненциальное сглаживание акселерометра
        expX = expFilter(expX, ax, FILTER_ALPHA);
        expY = expFilter(expY, ay, FILTER_ALPHA);
        expZ = expFilter(expZ, az, FILTER_ALPHA);
        
        // Определение вертикальной оси для корректного расчёта углов
        uint8_t v = detectVerticalAxis(expX, expY, expZ, &verticalAxisSign);
        
        // Расчёт углов наклона (используется в основном цикле)
        int16_t tx, ty;
        calcTilt(expX, expY, expZ, &tx, &ty, v, verticalAxisSign);
        
        sumTX += tx;
        sumTY += ty;
        samples++;
        
        // Обратный отсчёт
        uint8_t sec = (millis() - startTime) / 1000;
        if (sec < CALIB_DURATION / 1000 && (millis() - startTime) % 1000 < 50) {
            Serial.print(CALIB_DURATION / 1000 - sec);
            Serial.println(F("s"));
        }
        
        // Отмена кнопкой
        if (digitalRead(BUTTON_PIN) == LOW) {
            delay(50);
            if (digitalRead(BUTTON_PIN) == LOW) {
                Serial.println(F("CANCELLED"));
                calibrating = false;
                while(digitalRead(BUTTON_PIN) == LOW) delay(10);
                return;
            }
        }
        delay(20);
    }
    
    // Сохранение опорных углов
    if (samples > 0) {
        refTiltX = sumTX / samples;
        refTiltY = sumTY / samples;
        hasCalibration = true;
        lastTiltX = 32767;
        lastTiltY = 32767;
        resetStability();
        Serial.print(F("Zero set: X="));
        Serial.print(refTiltX / 100.0, 2);
        Serial.print(F(" Y="));
        Serial.println(refTiltY / 100.0, 2);
    }
    Serial.println(F("=== DONE ===\n"));
    calibrating = false;
}

// ============================================================================
// Вспомогательное
// ============================================================================

void printDegrees(int16_t deg) {
    Serial.print(deg / 100);
    Serial.print(F("."));
    int16_t frac = abs(deg % 100);
    if (frac < 10) Serial.print(F("0"));
    Serial.print(frac);
}

int32_t calcAltitude(int32_t p) {
    if (p <= 0) return 0;
    return (int32_t)(44330.0 * (1.0 - pow((float)p / (float)basePressure, 0.190295))) * 1000;
}

void setBaroCalibration(int32_t knownAltitudeM) {
    if (!baroCalibrated) return;
    float ratio = 1.0 - (float)knownAltitudeM / 44330.0;
    basePressure = (int32_t)((float)lastBaroP / pow(ratio, 5.255));
    Serial.print(F("  P0="));
    Serial.print(basePressure);
    Serial.print(F(" Pa (H="));
    Serial.print(knownAltitudeM);
    Serial.print(F("m), P="));
    Serial.print(lastBaroP);
    Serial.println(F(" Pa"));
}

bool tiltChanged(int16_t x, int16_t y) {
    if (abs(x - lastTiltX) > TILT_THRESHOLD || abs(y - lastTiltY) > TILT_THRESHOLD) {
        lastTiltX = x;
        lastTiltY = y;
        return true;
    }
    return false;
}

// ============================================================================
// SETUP & LOOP
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Wire.begin();
    
    // Инициализация кнопки с внешним прерыванием
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, CHANGE);

    initMPU6050();
    initHMC5883L();
    initMS5611();

    if (baroCalibrated) {
        readMS5611();
        setBaroCalibration(180);
    }

    Serial.println(F("\n=== GY-86 Инклинометр v9.1 (UART версия) ==="));
    Serial.println(F("Нажмите кнопку для управления:"));
    Serial.println(F("1x: Переключение режима отображения"));
    Serial.println(F("2x: Переключение градусов/сырые данные"));
    Serial.println(F("3x/Long: Калибровка нуля"));
    Serial.println(F("============================================\n"));
}

void loop() {
    // Опрос датчиков с заданным интервалом
    if (millis() - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
        int16_t ax, ay, az;
        readMPU6050Filtered(&ax, &ay, &az);
        readMPU6050Gyro(&gyroX, &gyroY, &gyroZ);
        readHMC5883L();
        
        if (filterInit) {
            expX = expFilter(expX, ax, FILTER_ALPHA);
            expY = expFilter(expY, ay, FILTER_ALPHA);
            expZ = expFilter(expZ, az, FILTER_ALPHA);
        } else {
            expX = ax;
            expY = ay;
            expZ = az;
            filterInit = true;
        }
        
        verticalAxis = detectVerticalAxis(expX, expY, expZ, &verticalAxisSign);
        
        if (verticalAxis != prevVerticalAxis) {
            prevVerticalAxis = verticalAxis;
            lastTiltX = 32767;
            lastTiltY = 32767;
            resetStability();
            avgIndex = 0;
            avgReady = false;
        }
        
        int16_t tX, tY;
        calcTilt(expX, expY, expZ, &tX, &tY, verticalAxis, verticalAxisSign);

        // Применение калибровки нуля (если выполнена)
        if (hasCalibration) {
            tX -= refTiltX;
            tY -= refTiltY;
        }

        addTiltSample(tX, tY);
        if (!avgReady && avgIndex == 0) avgReady = true;
        checkStability(tX, tY);
        
        lastSensorReadTime = millis();
    }
    
    // Обработка нажатий кнопки (флаг из прерывания)
    uint8_t btn = readButton();
    if (btn == 1) { 
        displayMode = (displayMode + 1) % 2; 
        Serial.print(F("\nРежим отображения: "));
        Serial.println(displayMode == 0 ? F("Нормальный") : F("Упрощенный"));
    }
    else if (btn == 2) { 
        useDegrees = !useDegrees; 
        Serial.print(F("\nРежим вывода: "));
        Serial.println(useDegrees ? F("Градусы") : F("Сырые данные"));
    }
    else if (btn == 3) { 
        runCalibration(); 
    }
    else if (btn == 4) { 
        runCalibration(); 
    }
    
    if (calibrating) return;
    
    // Постоянный вывод в UART с увеличенным интервалом
    if (millis() - lastOutputTime >= UART_UPDATE_INTERVAL) {
        lastOutputTime = millis();
        static uint8_t slowCycle = 0;
        if (++slowCycle >= 5) {  // Уменьшена частота опроса барометра
            readMS5611();
            slowCycle = 0;
        }
        
        int16_t avgTX, avgTY;
        getRobustAvgTilt(&avgTX, &avgTY);
        
        if (useDegrees) {
            // Вывод в градусах
            if (displayMode == 0) {
                // Нормальный режим
                Serial.print(F("=== ДАННЫЕ ===\n"));
                Serial.print(F("Ось: "));
                Serial.print(verticalAxis == 0 ? F("X") : verticalAxis == 1 ? F("Y") : F("Z"));
                Serial.print(verticalAxisSign > 0 ? F("+") : F("-"));
                Serial.print(F("\nУглы:\n  X: "));
                printDegrees(avgTX);
                Serial.print(F("°\n  Y: "));
                printDegrees(avgTY);
                Serial.print(F("°\n"));
                
                if (isStable) {
                    Serial.print(F("Состояние: [СТАБИЛЬНО]\n"));
                } else {
                    Serial.print(F("Состояние: [В ДВИЖЕНИИ]\n"));
                }
                
                if (baroCalibrated) {
                    Serial.print(F("Давление: "));
                    Serial.print(lastBaroP / 100);
                    Serial.print(F(" Па\n"));
                    Serial.print(F("Высота: "));
                    Serial.print(calcAltitude(lastBaroP) / 1000);
                    Serial.print(F(" м\n"));
                    Serial.print(F("Температура: "));
                    Serial.print(lastBaroT / 100.0, 1);
                    Serial.print(F(" °C\n"));
                }
                
                Serial.print(F("Компас: "));
                if (magAvailable) {
                    Serial.print(atan2((float)magY, (float)magX) * 5729.57795 / 100);
                    Serial.print(F("°\n"));
                } else {
                    Serial.print(F("[Недоступен]\n"));
                }
                
            } else {
                // Упрощенный режим
                Serial.print(F("V:"));
                Serial.print(verticalAxis == 0 ? F("X") : verticalAxis == 1 ? F("Y") : F("Z"));
                Serial.print(verticalAxisSign > 0 ? F("+") : F("-"));
                Serial.print(F(" X:"));
                printDegrees(avgTX);
                Serial.print(F(" Y:"));
                printDegrees(avgTY);
                if (isStable) Serial.print(F(" [TRUE]"));
                if (baroCalibrated) {
                    Serial.print(F(" P:"));
                    Serial.print(lastBaroP / 100);
                    Serial.print(F(" H:"));
                    Serial.print(calcAltitude(lastBaroP) / 1000);
                    Serial.print(F("m T:"));
                    Serial.print(lastBaroT / 100.0, 1);
                    Serial.print(F("C"));
                }
                Serial.println();
            }
        } else {
            // Вывод сырых данных
            if (displayMode == 0) {
                // Нормальный режим
                Serial.print(F("=== СЫРЫЕ ДАННЫЕ ===\n"));
                Serial.print(F("Акселерометр: "));
                Serial.print(expX);
                Serial.print(F(", "));
                Serial.print(expY);
                Serial.print(F(", "));
                Serial.print(expZ);
                Serial.print(F("\n"));
                
                Serial.print(F("Гироскоп: "));
                Serial.print(gyroX);
                Serial.print(F(", "));
                Serial.print(gyroY);
                Serial.print(F(", "));
                Serial.print(gyroZ);
                Serial.print(F("\n"));
                
                Serial.print(F("Магнитометр: "));
                Serial.print(magX);
                Serial.print(F(", "));
                Serial.print(magY);
                Serial.print(F(", "));
                Serial.print(magZ);
                Serial.print(F("\n"));
                
                if (baroCalibrated) {
                    Serial.print(F("Барометр: P="));
                    Serial.print(lastBaroP);
                    Serial.print(F(", T="));
                    Serial.print(lastBaroT);
                    Serial.print(F("\n"));
                }
                
            } else {
                // Упрощенный режим
                Serial.print(F("A:"));
                Serial.print(expX);
                Serial.print(F(","));
                Serial.print(expY);
                Serial.print(F(","));
                Serial.print(expZ);
                Serial.print(F(" V:"));
                Serial.print(verticalAxis == 0 ? F("X") : verticalAxis == 1 ? F("Y") : F("Z"));
                Serial.print(verticalAxisSign > 0 ? F("+") : F("-"));
                Serial.print(F(" G:"));
                Serial.print(gyroX);
                Serial.print(F(","));
                Serial.print(gyroY);
                Serial.print(F(","));
                Serial.print(gyroZ);
                Serial.print(F(" M:"));
                Serial.print(magX);
                Serial.print(F(","));
                Serial.print(magY);
                Serial.print(F(","));
                Serial.print(magZ);
                if (baroCalibrated) {
                    Serial.print(F(" B:"));
                    Serial.print(lastBaroP);
                    Serial.print(F(","));
                    Serial.print(lastBaroT);
                }
                Serial.println();
            }
        }
        
        Serial.println(F("============================================"));
    }
    
    delay(1);
}