#include <Arduino.h>

#define POTENTIOMETER_PIN A3                    // пин подключения потенциометра
#define INDICATOR_PIN A2                        // пин подключения индикатора

#define ADC_MAXIMUM_VALUE 1023                 // максимальное значение АЦП для Arduino Nano (10 бит = 0-1023)
#define VOLTAGE_SEGMENTS_COUNT 6                // количество сегментов деления напряжения
#define ADC_MINIMUM_NOISE_THRESHOLD 1          // порог отсечки шумов в начале (нестабильная зона)
#define ADC_MAXIMUM_WORKING_THRESHOLD 700       // верхняя граница для 10 сегмента (реальное макс. напряжение)

int lastStableLevel = -1;                      // последнее стабильное значение уровня (0-10)
unsigned long transitionStartTime = 0;         // время начала мигания
bool isIndicatorFlashing = false;              // флаг активного мигания
int targetIntensity = 0;                       // целевой уровень после перехода

// Функция определения уровня с учётом нестабильных зон
int calculate_voltage_level(int adc_value) {
    // Зона 0: adc_value <= ADC_MINIMUM_NOISE_THRESHOLD - выключено
    if (adc_value <= ADC_MINIMUM_NOISE_THRESHOLD) {
        return 0;
    }

    // Зона 1-6: распределяем диапазон от порога до максимума
    // Каждый сегмент = (ADC_MAXIMUM_WORKING_THRESHOLD - ADC_MINIMUM_NOISE_THRESHOLD) / 6
    int range = ADC_MAXIMUM_WORKING_THRESHOLD - ADC_MINIMUM_NOISE_THRESHOLD;
    int segment_size = range / VOLTAGE_SEGMENTS_COUNT;

    int adjusted_value = adc_value - ADC_MINIMUM_NOISE_THRESHOLD;
    int level = adjusted_value / segment_size + 1;

    // Ограничиваем уровень максимумом 6
    if (level > VOLTAGE_SEGMENTS_COUNT) {
        level = VOLTAGE_SEGMENTS_COUNT;
    }

    return level;
}

void initialize_hardware() {
    pinMode(POTENTIOMETER_PIN, INPUT);
    pinMode(INDICATOR_PIN, OUTPUT);
    digitalWrite(INDICATOR_PIN, LOW);

    Serial.begin(115200);

    // Увеличиваем скорость АЦП
    // Делитель 64 = 250 кГц (быстрее стандартных 125 кГц)
    ADCSRA = (ADCSRA & 0xF8) | 0x06;
}

void main_control_cycle() {
    // Читаем аналоговое значение с потенциометра (0-1023)
    int adc_value = analogRead(POTENTIOMETER_PIN);

    // Вычисляем уровень с учётом нестабильных зон
    int level = calculate_voltage_level(adc_value);

    unsigned long current_time = millis();

    // Обработка мигания (одиночный импульс при переходе)
    if (isIndicatorFlashing) {
        if (current_time - transitionStartTime >= 150) {
            // Мигание завершено (150 мс)
            isIndicatorFlashing = false;
            
            // После мигания устанавливаем индикатор по целевому уровню
            if (targetIntensity > 0) {
                digitalWrite(INDICATOR_PIN, HIGH);
            } else {
                digitalWrite(INDICATOR_PIN, LOW);
            }
        } else if (current_time - transitionStartTime >= 75) {
            // Вторая фаза: выключаем индикатор (75-150 мс)
            digitalWrite(INDICATOR_PIN, LOW);
        }
        // Первая фаза (0-75 мс): индикатор включён
    }

    // Детектирование перехода
    if (!isIndicatorFlashing && lastStableLevel >= 0) {
        // Переход на другой уровень
        if (level != lastStableLevel) {
            targetIntensity = level;
            
            // Запускаем одиночное мигание
            isIndicatorFlashing = true;
            transitionStartTime = current_time;
            digitalWrite(INDICATOR_PIN, HIGH); // Первая фаза: включаем

            lastStableLevel = level;

            // Выводим текущее значение уровня в UART
            Serial.print(current_time);
            Serial.print(",LEVEL=");
            Serial.println(level);
        }
    } else if (lastStableLevel < 0) {
        // Инициализация первого значения
        lastStableLevel = level;
        targetIntensity = level;

        // Устанавливаем начальное состояние индикатора
        if (level > 0) {
            digitalWrite(INDICATOR_PIN, HIGH);
        } else {
            digitalWrite(INDICATOR_PIN, LOW);
        }
    }

    delay(5); // минимальная задержка для стабильности чтения
}

void setup() {
    initialize_hardware();
}

void loop() {
    main_control_cycle();
}