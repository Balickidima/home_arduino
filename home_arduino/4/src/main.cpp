#include <Arduino.h>

#define SWITCH_PIN A1
#define INDICATOR_PIN A2
#define CONTACT_DEBOUNCE_INTERVAL_MS 50  // задержка подавления дребезга контакта, мс
const int BLINK_INTERVAL = 100;         // фиксированный интервал мерцания (мс)

// Константы для 74HC595
#define DATA_PIN 2      // DS (Data) - пин данных
#define CLOCK_PIN 3     // SH_CP (Clock) - пин тактового сигнала
#define LATCH_PIN 4     // ST_CP (Latch) - пин защелки

// Переменные для управления светодиодами
bool currentSwitchRawState = HIGH;
bool lastStableSwitchState = HIGH;      // последнее стабильное состояние переключателя
unsigned long lastStateChangeTimestamp = 0; // время последнего изменения состояния
bool isBlinking = false;                 // флаг включения мерцания
unsigned long lastBlinkTime = 0;         // время последнего переключения состояния светодиода
int currentPattern = 0;                  // текущий паттерн индикации
unsigned long lastPatternChangeTime = 0; // время последнего изменения паттерна
const int PATTERN_CHANGE_INTERVAL = 3000; // интервал смены паттерна (3 секунды)
int currentMode = 0;                     // текущий режим работы

// Функция для отправки бита в 74HC595
void shiftOutBit(bool bitValue) {
    digitalWrite(DATA_PIN, bitValue);
    digitalWrite(CLOCK_PIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(CLOCK_PIN, LOW);
    delayMicroseconds(1);
}

// Функция для отправки байта в 74HC595 (MSB first)
void shiftOutByte(byte data) {
    for (int i = 7; i >= 0; i--) {
        bool bitValue = (data >> i) & 1;
        shiftOutBit(bitValue);
    }
}

// Функция для обновления состояния светодиодов
void updateLEDs(byte pattern) {
    digitalWrite(LATCH_PIN, LOW);
    shiftOutByte(pattern);
    digitalWrite(LATCH_PIN, HIGH);
}

// Функция для включения N светодиодов (слева направо)
void turnOnLEDs(int count) {
    if (count < 0) count = 0;
    if (count > 8) count = 8;
    byte pattern = 0;
    for (int i = 0; i < count; i++) {
        pattern |= (1 << i);
    }
    updateLEDs(pattern);
}

// Функция для создания бегущего огня
void runningLight(int position) {
    byte pattern = 1 << (position % 8);
    updateLEDs(pattern);
}

// Функция для создания последовательности индикации
void sequencePattern(int seqNum) {
    byte patterns[8][8] = {
        {0b00000001, 0b00000010, 0b00000100, 0b00001000, 0b00010000, 0b00100000, 0b01000000, 0b10000000}, // Бегущий огонь
        {0b00000011, 0b00000110, 0b00001100, 0b00011000, 0b00110000, 0b01100000, 0b11000000, 0b10000001}, // Бегущий огонь 2
        {0b00001111, 0b00111100, 0b11110000, 0b11110000, 0b00111100, 0b00001111, 0b00000000, 0b00000000}, // Пульсация
        {0b01010101, 0b10101010, 0b01010101, 0b10101010, 0b01010101, 0b10101010, 0b01010101, 0b10101010}, // Шахматная доска
        {0b11111111, 0b00000000, 0b11111111, 0b00000000, 0b11111111, 0b00000000, 0b11111111, 0b00000000}, // Мигание
        {0b10000001, 0b01000010, 0b00100100, 0b00011000, 0b00011000, 0b00100100, 0b01000010, 0b10000001}, // Ромб
        {0b11111111, 0b01111110, 0b00111100, 0b00011000, 0b00011000, 0b00111100, 0b01111110, 0b11111111}, // Овал
        {0b00011110, 0b00111100, 0b01111000, 0b11110000, 0b11110000, 0b01111000, 0b00111100, 0b00011110}  // Квадрат
    };
    
    if (seqNum >= 0 && seqNum < 8) {
        updateLEDs(patterns[seqNum][currentPattern % 8]);
        currentPattern++;
    }
}

void setup() {
    pinMode(SWITCH_PIN, INPUT_PULLUP);
    pinMode(INDICATOR_PIN, OUTPUT);
    pinMode(DATA_PIN, OUTPUT);
    pinMode(CLOCK_PIN, OUTPUT);
    pinMode(LATCH_PIN, OUTPUT);
    
    digitalWrite(INDICATOR_PIN, LOW);
    digitalWrite(DATA_PIN, LOW);
    digitalWrite(CLOCK_PIN, LOW);
    digitalWrite(LATCH_PIN, LOW);

    Serial.begin(115200);
    Serial.println("System initialized with 74HC595");
    Serial.println("Press button to change patterns");
    
    // Начальное состояние - 4 светодиода
    turnOnLEDs(4);
}

void loop() {
    bool button_state = digitalRead(SWITCH_PIN);
    unsigned long current_time = millis();

    // Обработка кнопки
    if (button_state != lastStableSwitchState) {
        // Если прошло достаточно времени — это не дребезг
        if (current_time - lastStateChangeTimestamp > CONTACT_DEBOUNCE_INTERVAL_MS) {
            lastStableSwitchState = button_state;
            lastStateChangeTimestamp = current_time;

            // Обработка нажатия (фронт: HIGH → LOW)
            if (button_state == LOW) {
                // Переключаем режим
                currentMode = (currentMode + 1) % 4;
                
                switch(currentMode) {
                    case 0:
                        Serial.println("Mode 0: 4 LEDs");
                        turnOnLEDs(4);
                        break;
                    case 1:
                        Serial.println("Mode 1: 5 LEDs");
                        turnOnLEDs(5);
                        break;
                    case 2:
                        Serial.println("Mode 2: Running light");
                        currentPattern = 0;
                        break;
                    case 3:
                        Serial.println("Mode 3: Sequence pattern");
                        currentPattern = 0;
                        break;
                }
            }
        } else {
            // Дребезг! Сообщаем в монитор порта
            Serial.println("BOUNCE_DETECTED");
        }
    }
    
    // Автоматическая смена паттернов
    if (current_time - lastPatternChangeTime >= PATTERN_CHANGE_INTERVAL) {
        lastPatternChangeTime = current_time;
        
        if (currentMode == 2) {
            // Режим бегущего огня
            runningLight(currentPattern);
            currentPattern++;
        } else if (currentMode == 3) {
            // Режим последовательности
            sequencePattern(currentMode);
        }
    }

    currentSwitchRawState = button_state;
    delay(10); // Небольшая задержка для стабильности
}