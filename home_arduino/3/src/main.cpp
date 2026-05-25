#include <Arduino.h>

#define SWITCH_PIN A1
#define INDICATOR_PIN A2
#define POTENTIOMETER_PIN 3              // пин подключения потенциометра
#define MODE_SWITCH_PIN 2                // пин кнопки переключения режима
#define CONTACT_DEBOUNCE_INTERVAL_MS 50  // задержка подавления дребезга контакта, мс

// Константы для калибровки потенциометра
const int POTENTIOMETER_MIN = 0;        // минимальное значение АЦП
const int POTENTIOMETER_MAX = 1023;     // максимальное значение АЦП
const int BLINK_MIN_INTERVAL = 8;       // минимальный интервал мерцания (мс) - частое мерцание (50/6)
const int BLINK_MAX_INTERVAL = 166;    // максимальный интервал мерцания (мс) - редкое мерцание (1000/6)

bool indicatorStatus = false;
bool currentSwitchRawState = HIGH;
bool lastStableSwitchState = HIGH;      // последнее стабильное состояние переключателя
unsigned long lastStateChangeTimestamp = 0; // время последнего изменения состояния
bool isBlinking = false;                 // флаг включения мерцания
unsigned long lastBlinkTime = 0;         // время последнего переключения состояния светодиода
int currentBlinkInterval = 0;            // текущий интервал мерцания
bool isMonitoringMode = false;           // флаг режима мониторинга
unsigned long buttonPressStartTime = 0;   // время начала нажатия
const unsigned long LONG_PRESS_DURATION = 5000; // длительность долгого нажатия (5 секунд)
unsigned long lastPotentiometerReadTime = 0; // время последнего чтения с потенциометром
const unsigned long POTENTIOMETER_READ_INTERVAL = 1000; // интервал чтения (1 секунда)

void setup() {
    pinMode(SWITCH_PIN, INPUT_PULLUP);
    pinMode(INDICATOR_PIN, OUTPUT);
    pinMode(POTENTIOMETER_PIN, INPUT);
    pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
    digitalWrite(INDICATOR_PIN, LOW);

    Serial.begin(115200);
    Serial.println("System initialized");
    Serial.print("Initial mode: ");
    Serial.println(isMonitoringMode ? "MONITORING" : "NORMAL");
    
    // Проверка диапазона потенциометра
    int minVal = 1023, maxVal = 0;
    for (int i = 0; i < 10; i++) {
        int val = analogRead(POTENTIOMETER_PIN);
        if (val < minVal) minVal = val;
        if (val > maxVal) maxVal = val;
        delay(10);
    }
    
    Serial.print("Potentiometer range: ");
    Serial.print(minVal);
    Serial.print(" - ");
    Serial.println(maxVal);
    
    // Если диапазон сильно отличается от ожидаемого, выводим предупреждение
    if (maxVal - minVal < 800) {
        Serial.println("WARNING: Potentiometer range is too small!");
    }
}

void loop() {
    bool button_state = digitalRead(SWITCH_PIN);
    bool mode_button_state = digitalRead(MODE_SWITCH_PIN);
    unsigned long current_time = millis();

    // Обработка кнопки переключения режима
    if (mode_button_state == LOW) {
        // Кнопка нажата
        if (buttonPressStartTime == 0) {
            // Кнопка только что нажата
            buttonPressStartTime = current_time;
            Serial.println("Mode button pressed");
        } else {
            // Кнопка удерживается
            unsigned long pressDuration = current_time - buttonPressStartTime;
            
            // Проверяем, нужно ли переключить режим
            if (pressDuration >= LONG_PRESS_DURATION) {
                // Долгое нажатие - переключаем режим
                isMonitoringMode = !isMonitoringMode;
                Serial.print("Mode switched to: ");
                Serial.println(isMonitoringMode ? "MONITORING" : "NORMAL");
                
                // Визуальная индикация переключения - 3 быстрых мигания
                for (int i = 0; i < 3; i++) {
                    digitalWrite(INDICATOR_PIN, HIGH);
                    delay(100);
                    digitalWrite(INDICATOR_PIN, LOW);
                    delay(100);
                }
                
                // Сбрасываем время нажатия
                buttonPressStartTime = 0;
            }
        }
    } else {
        // Кнопка не нажата
        if (buttonPressStartTime > 0) {
            Serial.println("Mode button released before timeout");
            buttonPressStartTime = 0;
        }
    }

    // Если в режиме мониторинга, выводим значение с потенциометра
    if (isMonitoringMode) {
        // Проверяем, прошло ли достаточно времени с последнего чтения
        if (current_time - lastPotentiometerReadTime >= POTENTIOMETER_READ_INTERVAL) {
            int potentiometerValue = analogRead(POTENTIOMETER_PIN);
            
            // Преобразуем значение в интервал мерцания (обратная зависимость)
            currentBlinkInterval = map(potentiometerValue, POTENTIOMETER_MIN, POTENTIOMETER_MAX, BLINK_MIN_INTERVAL, BLINK_MAX_INTERVAL);
            
            // Ограничиваем значение в допустимом диапазоне
            currentBlinkInterval = constrain(currentBlinkInterval, BLINK_MIN_INTERVAL, BLINK_MAX_INTERVAL);
            
            Serial.print("POTENTIOMETER_RAW=");
            Serial.print(potentiometerValue);
            Serial.print(" | BLINK_INTERVAL=");
            Serial.print(currentBlinkInterval);
            Serial.println("ms");
            
            lastPotentiometerReadTime = current_time;
        }
    } else {
        // Обработка обычной кнопки только в нормальном режиме
        if (button_state != lastStableSwitchState) {
            // Если прошло достаточно времени — это не дребезг
            if (current_time - lastStateChangeTimestamp > CONTACT_DEBOUNCE_INTERVAL_MS) {
                lastStableSwitchState = button_state;
                lastStateChangeTimestamp = current_time;

                // Обработка нажатия (фронт: HIGH → LOW)
                if (button_state == LOW) {
                    // Переключаем состояние мерцания
                    isBlinking = !isBlinking;
                    
                    if (isBlinking) {
                        // При включении мерцания читаем значение с потенциометра
                        int potentiometerValue = analogRead(POTENTIOMETER_PIN);
                        
                        // Преобразуем значение в интервал мерцания (обратная зависимость)
                        currentBlinkInterval = map(potentiometerValue, POTENTIOMETER_MIN, POTENTIOMETER_MAX, BLINK_MIN_INTERVAL, BLINK_MAX_INTERVAL);
                        
                        // Ограничиваем значение в допустимом диапазоне
                        currentBlinkInterval = constrain(currentBlinkInterval, BLINK_MIN_INTERVAL, BLINK_MAX_INTERVAL);
                        
                        Serial.print("BLINKING_ENABLED,INTERVAL=");
                        Serial.print(currentBlinkInterval);
                        Serial.println("ms");
                    } else {
                        // При выключении гасим светодиод
                        digitalWrite(INDICATOR_PIN, LOW);
                        Serial.println("BLINKING_DISABLED");
                    }
                }
            } else {
                // Дребезг! Сообщаем в монитор порта
                Serial.println("BOUNCE_DETECTED");
            }
        }
        
        // Обработка мерцания (только если включено)
        if (isBlinking) {
            // Проверяем, прошло ли достаточно времени с последнего переключения
            // Используем unsigned long для обоих операндов сравнения
            if ((current_time - lastBlinkTime) >= (unsigned long)currentBlinkInterval) {
                // Переключаем состояние светодиода
                indicatorStatus = !indicatorStatus;
                digitalWrite(INDICATOR_PIN, indicatorStatus ? HIGH : LOW);
                
                // Обновляем время последнего переключения
                lastBlinkTime = current_time;
            }
        }
    }

    currentSwitchRawState = button_state;
    delay(10); // Небольшая задержка для стабильности
}