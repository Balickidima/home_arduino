#include <Arduino.h>

#define SWITCH_PIN A1
#define INDICATOR_PIN A2
#define CONTACT_DEBOUNCE_INTERVAL_MS 50  // задержка подавления дребезга контакта, мс

bool indicatorStatus = false;
bool currentSwitchRawState = HIGH;
bool lastStableSwitchState = HIGH;      // последнее стабильное состояние переключателя
unsigned long lastStateChangeTimestamp = 0; // время последнего изменения состояния

void setup() {
    pinMode(SWITCH_PIN, INPUT_PULLUP);
    pinMode(INDICATOR_PIN, OUTPUT);
    digitalWrite(INDICATOR_PIN, LOW);

    Serial.begin(115200);
}

void loop() {
    bool button_state = digitalRead(SWITCH_PIN);
    unsigned long current_time = millis();

    // Детектируем изменение состояния переключателя
    if (button_state != lastStableSwitchState) {
        // Если прошло достаточно времени — это не дребезг
        if (current_time - lastStateChangeTimestamp > CONTACT_DEBOUNCE_INTERVAL_MS) {
            lastStableSwitchState = button_state;
            lastStateChangeTimestamp = current_time;

            // Обработка нажатия (фронт: HIGH → LOW)
            if (button_state == LOW) {
                indicatorStatus = !indicatorStatus;
                digitalWrite(INDICATOR_PIN, indicatorStatus ? HIGH : LOW);
                Serial.print(current_time);
                Serial.print(",BTN_PRESSED,LED=");
                Serial.println(indicatorStatus ? 1 : 0);
            }
        } else {
            // Дребезг! Сообщаем в монитор порта
            Serial.print(current_time);
            Serial.println(",BOUNCE_DETECTED");
        }
    }

    currentSwitchRawState = button_state;
}