#include <Wire.h> // I2C communication (https://github.com/arduino/ArduinoCore-avr/tree/master/libraries/Wire)
#include <RTClib.h> // RTC (date & time) (https://github.com/adafruit/RTClib)
#include <SD.h> // Read/write SD card (https://github.com/arduino-libraries/SD)
#include <SPI.h> // SPI communication (https://github.com/arduino/ArduinoCore-avr/tree/master/libraries/SPI)
#include <BsecSensor.h> // BME680 sensor + air quality (IAQ) (https://github.com/BoschSensortec/BSEC-Arduino-library)
#include "ESP32-HUB75-MatrixPanel-I2S-DMA.h" // LED matrix display via I2S (https://github.com/mrfaptastic/ESP32-HUB75-MatrixPanel-I2S-DMA)
#include "MatrixPanelConfig.h" // LED matrix configuration
#include "icons.h" // Icon graphics for LED matrix
#include "GFX_fonts/Font5x7FixedMono.h" // 5x7 fixed-width font (https://github.com/robjen/GFX_fonts.git)
#include "Fonts/TomThumb.h" // Tiny font (https://github.com/adafruit/Adafruit-GFX-Library/tree/master/Fonts)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // ESP32 multitasking (FreeRTOS)(https://github.com/espressif/esp-idf/tree/master/components/freertos)

MatrixPanel_I2S_DMA *display;   // LED matrix display object
SemaphoreHandle_t xMutex;       // Mutex to protect access to sensorData
BsecSensor sensor;              // BME680 sensor object
RTC_DS1307 rtc;                 // RTC object   
File dataFile;                  // File object for SD card

// Variables to store sensor data accumulation
float temperatureSum = 0.0;
int humiditySum = 0;
float pressureSum = 0.0;
int iaqSum = 0;
int co2Sum = 0;
float vocSum = 0.0;
float latestIAQ = 0.0;
int readingCount = 0;
unsigned long lastSaveTime = 0;

#define SD_CS_PIN 5 // SD card chip select pin

#define BUZZER_PIN        32  // Buzzer control pin
#define BUZZER_CHANNEL     0  // PWM channel for buzzer
#define BUZZER_FREQ     1000  // Buzzer frequency: 1 kHz
#define BUZZER_RESOLUTION  8  // PWM resolution: 8-bit (0–255)
#define BUZZER_DUTY_ON   128  // 50% duty cycle (buzzer ON)
#define BUZZER_DUTY_OFF    0  // 0% duty cycle (buzzer OFF)

// Variables for color
uint16_t myBLACK, myWHITE, myRED, myGREEN, myBLUE, myYELLOW, myORANGE, myCYAN, myMAGENTA;
uint16_t myDARKGRAY, myLIGHTGRAY, myPURPLE, myPINK, myBROWN, myLIME, myTEAL, myNAVY;

void initColors(MatrixPanel_I2S_DMA* display) {
    // Initialize color with color565()
    myBLACK = display->color565(0, 0, 0);
    myWHITE = display->color565(255, 255, 255);
    myRED = display->color565(255, 0, 0);
    myGREEN = display->color565(0, 255, 0);
    myBLUE = display->color565(0, 0, 255);
    myYELLOW = display->color565(255, 255, 0);
    myORANGE = display->color565(255, 165, 0);
    myCYAN = display->color565(0, 255, 255);
    myMAGENTA = display->color565(255, 0, 255);
    myDARKGRAY = display->color565(169, 169, 169);
    myLIGHTGRAY = display->color565(211, 211, 211);
    myPURPLE = display->color565(128, 0, 128);
    myPINK = display->color565(255, 105, 180);
    myBROWN = display->color565(139, 69, 19);
    myLIME = display->color565(0, 255, 0);
    myTEAL = display->color565(0, 128, 128);
    myNAVY = display->color565(0, 0, 128);
}
// Task handles for FreeRTOS tasks
TaskHandle_t readSensorTaskHandle = NULL;   // Task to read sensor data
TaskHandle_t printDataTaskHandle = NULL;    // Task to print data to Serial
TaskHandle_t saveDataTaskHandle = NULL;     // Task to save data to SD card
TaskHandle_t printPanelTaskHandle = NULL;   // Task to display data on LED panel

// Task to print timestamp and sensor data
void printDataTask(void *pvParameters) {
    for (;;) {
        DateTime now = rtc.now(); // Get current time from RTC

        // Print timestamp in format: YYYY-MM-DD HH:MM:SS
        Serial.print("Timestamp: ");
        Serial.print(now.year()); Serial.print("-");
        Serial.print(now.month() < 10 ? "0" : ""); Serial.print(now.month()); Serial.print("-");
        Serial.print(now.day() < 10 ? "0" : ""); Serial.print(now.day()); Serial.print(" ");
        Serial.print(now.hour() < 10 ? "0" : ""); Serial.print(now.hour()); Serial.print(":");
        Serial.print(now.minute() < 10 ? "0" : ""); Serial.print(now.minute()); Serial.print(":");
        Serial.print(now.second() < 10 ? "0" : ""); Serial.println(now.second());

        // Print sensor data using BsecSensor method
        sensor.printData();

        vTaskDelay(2000 / portTICK_PERIOD_MS); // Delay 2 seconds
    }
}

// Task to read sensor data and accumulate values
void readSensorTask(void *pvParameters) {
    while (true) {
        // Take mutex before accessing shared sensor data
        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
            sensor.update(); // Update sensor readings

            // Add current readings to totals
            temperatureSum += sensor.getTemperature();
            humiditySum += sensor.getHumidity();
            pressureSum += sensor.getPressure();
            iaqSum += sensor.getIAQ();
            co2Sum += sensor.getCarbon();
            vocSum += sensor.getVOC();
            readingCount++;

            latestIAQ = sensor.getIAQ(); // Store latest IAQ reading

            xSemaphoreGive(xMutex); // Release mutex
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Delay 2 seconds
    }
}

// Task to play alarm repeatedly if IAQ is high
void alarmTask(void *pvParameters) {
    float localIAQ = 0;

    while (true) {
        // Get the latest IAQ value safely
        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
            localIAQ = latestIAQ;
            xSemaphoreGive(xMutex);
        }

        Serial.print("Latest IAQ: ");
        Serial.println(localIAQ);

        // Trigger buzzer if IAQ > 150
        if (localIAQ > 150) {
            ledcWrite(BUZZER_CHANNEL, BUZZER_DUTY_ON);  // Turn buzzer ON
            vTaskDelay(pdMS_TO_TICKS(1000));            // Wait 1 second
            ledcWrite(BUZZER_CHANNEL, BUZZER_DUTY_OFF); // Turn buzzer OFF
            vTaskDelay(pdMS_TO_TICKS(1000));            // Wait 1 second
        } else {
            ledcWrite(BUZZER_CHANNEL, BUZZER_DUTY_OFF); // Keep buzzer OFF
            vTaskDelay(pdMS_TO_TICKS(1000));            // Wait 1 second
        }
    }
}

// Task to save averaged sensor data to SD card every hour
void saveDataTask(void *pvParameters) {
    while (true) {
        // Access shared data safely
        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
            unsigned long currentMillis = millis();

            // Save data every 1 hour (3600000 ms)
            if (currentMillis - lastSaveTime >= 3600000) {
                if (readingCount > 0) {
                    // Calculate average values
                    float avgTemperature = temperatureSum / readingCount;
                    float avgHumidity = humiditySum / readingCount;
                    float avgPressure = pressureSum / readingCount;
                    float avgIAQ = iaqSum / readingCount;
                    float avgCO2 = co2Sum / readingCount;
                    float avgVOC = vocSum / readingCount;

                    // Open file in append mode
                    dataFile = SD.open("/IAQdata_logger.csv", FILE_APPEND);
                    if (dataFile) {
                        // Write header if first save
                        if (lastSaveTime == 0) {
                            dataFile.println("Timestamp,Temperature,Humidity,Pressure,IAQ,CO2,VOC");
                        }

                        // Get current timestamp
                        DateTime now = rtc.now();
                        String timestamp = String(now.year()) + "-" + String(now.month()) + "-" + 
                                           String(now.day()) + " " + String(now.hour()) + ":" + 
                                           String(now.minute()) + ":" + String(now.second());

                        // Write data to CSV
                        dataFile.print(timestamp); dataFile.print(",");
                        dataFile.print(avgTemperature); dataFile.print(",");
                        dataFile.print(avgHumidity); dataFile.print(",");
                        dataFile.print(avgPressure); dataFile.print(",");
                        dataFile.print(avgIAQ); dataFile.print(",");
                        dataFile.print(avgCO2); dataFile.print(",");
                        dataFile.println(avgVOC);

                        dataFile.close(); // Close file
                        Serial.println("Data saved to SD card.");
                    } else {
                        Serial.println("Error opening data.csv");
                    }

                    // Reset totals and counter for next reading period
                    temperatureSum = 0.0;
                    humiditySum = 0;
                    pressureSum = 0.0;
                    iaqSum = 0;
                    co2Sum = 0;
                    vocSum = 0.0;
                    readingCount = 0;
                    lastSaveTime = currentMillis;
                }
            }
            xSemaphoreGive(xMutex); // Release mutex
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Wait 2 seconds
    }
}

// Display icon based on IAQ status with different colors
void displayWeatherIconBasedOnAQI() {
    String status = sensor.getIAQStatus();

    if (status == "Good") {
        display->drawXBitmap(117, 3, weather_icon_code_02n, 8, 8, myGREEN);
    } else if (status == "Average") {
        display->drawXBitmap(117, 3, weather_icon_code_02n, 8, 8, myYELLOW);
    } else if (status == "Little Bad") {
        display->drawXBitmap(117, 3, weather_icon_code_02n, 8, 8, myORANGE);
    } else if (status == "Bad") {
        display->drawXBitmap(117, 3, weather_icon_code_02n, 8, 8, myMAGENTA);
    } else if (status == "Worse") {
        display->drawXBitmap(117, 3, weather_icon_code_02n, 8, 8, myPURPLE);
    } else if (status == "Very Very Bad") {
        display->drawXBitmap(117, 3, weather_icon_code_02n, 8, 8, myRED);
    } else if (status == "Very Bad") {
        display->drawXBitmap(117, 3, weather_icon_code_02n, 8, 8, myBROWN);
    }

    display->flipDMABuffer(); // Refresh the LED panel
}

// Task function to display time on the LED panel
void displayTime() {      
    //if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {  
    display->setTextWrap(false);
    display->setFont(&TomThumb); 
    DateTime now = rtc.now();

    // Format time and date strings
    char hourStr[3], minStr[3], secStr[3], dateStr[11];
    snprintf(hourStr, sizeof(hourStr), "%02d", now.hour());
    snprintf(minStr, sizeof(minStr), "%02d", now.minute());
    snprintf(secStr, sizeof(secStr), "%02d", now.second());
    snprintf(dateStr, sizeof(dateStr), "%02d-%02d-%04d", now.day(), now.month(), now.year());

    display->clearScreen(); // clear the screen 

    display->drawRect(1, 1, 45, 31, myWHITE);
    display->drawLine(3, 22, 43, 22, myBLUE);

    // Show hours 
    display->setTextSize(3);
    display->setCursor(3, 20);
    display->setTextColor(myWHITE);
    display->print(hourStr);
    display->print(":");
    
    // Show minutes 
    display->setTextSize(2);
    display->setCursor(30, 13);
    display->setTextColor(myGREEN);
    display->print(minStr);
    
    // Show seconds
    display->setTextSize(1);
    display->setCursor(37, 20);
    display->setTextColor(myGREEN);
    display->print(secStr);
    
    // Show date
    display->setTextSize(1);
    display->setCursor(4, 29);
    display->setTextColor(myWHITE);
    display->print(dateStr);
    
    display->flipDMABuffer(); // Refresh display
    // xSemaphoreGive(xMutex); 
    // }
    //Serial.println("Waktu berhasil ditampilkan!");
}

// Task to display sensor data on the LED panel
void printPanel(void* parameter) {
    bool toggleAQI_VOC = true;
    uint32_t lastBlinkTime = millis();

    while (true) {
       // if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
            display->fillScreen(MatrixColors::BLACK(display));

            // Convert sensor data to strings for display
            char tempStr[10], humStr[10], pressStr[10], iaqStr[10], co2Str[10], vocStr[10];
            snprintf(tempStr, sizeof(tempStr), "%.1fC", sensor.getTemperature());
            snprintf(humStr, sizeof(humStr), "%d%%", sensor.getHumidity());
            snprintf(pressStr, sizeof(pressStr), "%.1fhPa", sensor.getPressure());
            snprintf(iaqStr, sizeof(iaqStr), "%d", sensor.getIAQ());
            snprintf(co2Str, sizeof(co2Str), "%d ppm", sensor.getCarbon());
            snprintf(vocStr, sizeof(vocStr), "%.1f ppm", sensor.getVOC());

        // display->drawLine(49, 10, 128, 10, myBLUE);
        // display->drawLine(49, 31, 128, 31, myBLUE);
        
        displayTime(); // Show time on the display
        displayWeatherIconBasedOnAQI(); // Show icon based on IAQ status

        // Set text properties
        display->setTextWrap(false);
        display->setFont(&Font5x7FixedMono);
        display->setTextSize(1);
        display->setTextColor(MatrixColors::WHITE(display));

        // Top and bottom lines
        display->setCursor(50, 5);
        display->setTextColor(MatrixColors::YELLOW(display));
        display->print("--------------------------------");
        display->setCursor(50, 34);
        display->setTextColor(MatrixColors::YELLOW(display));
        display->print("--------------------------------");

        //Blink text for AQI & VOC every 3 seconds
        display->setCursor(50, 10);
        display->setTextSize(1);
        if (toggleAQI_VOC) {
            display->setTextColor(MatrixColors::WHITE(display));
            display->print("AQI:");
            display->print(iaqStr);
        } else {
            display->setTextColor(MatrixColors::BLUE(display));
            display->print("VOC:");
            display->print(vocStr);
        }
        
        if (millis() - lastBlinkTime >= 3000) {
            toggleAQI_VOC = !toggleAQI_VOC;
            lastBlinkTime = millis();
        }

        // Show CO2
        display->setCursor(50, 19);
        display->setTextSize(1);
        display->setTextColor(MatrixColors::RED(display));
        display->print("CO2:");
        display->print(co2Str);

        // Show temperature and humidity
        display->setCursor(50, 28);
        display->setTextColor(display->color565(255, 165, 0)); // Custom orange
        display->print("T:");
        display->print(tempStr);
        display->print(" ");
        display->setTextColor(MatrixColors::CYAN(display));
        display->print("H:");
        display->print(humStr);

        display->flipDMABuffer(); // Refresh the display
        //xSemaphoreGive(xMutex);
    //}
        vTaskDelay(pdMS_TO_TICKS(50)); // Small delay for smooth refresh
    }
}

void setup() {
    Serial.begin(115200);
    delay(100);

    // Initialize I2C (SDA = 21, SCL = 22)
    Wire.begin(21, 22);

    // Setup PWM for buzzer
    ledcSetup(BUZZER_CHANNEL, BUZZER_FREQ, BUZZER_RESOLUTION);
    ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);

    // Initialize BME688 sensor
    sensor.begin();
    sensor.update(); // Initial reading to avoid invalid data

    // Create a mutex for shared resources
    xMutex = xSemaphoreCreateMutex();
    if (xMutex == NULL) {
        Serial.println("Failed to create mutex!");
        while (1);
    } else {
        Serial.println("Mutex created successfully.");
    }

    // Initialize LED matrix display
    display = MatrixPanelConfig::initDisplay();
    if (display) {
        Serial.println("Display initialized successfully!");
    } else {
        Serial.println("Failed to initialize display!");
        while (1);
    }
    display->fillScreen(MatrixColors::BLACK(display)); // Clear screen

    // Initialize RTC module
    if (!rtc.begin()) {
        Serial.println("RTC not found!");
        while (1);
    }

    // Optional: Set initial RTC time if needed
    // if (!rtc.isrunning()) {
    //     Serial.println("RTC not running. Setting initial time...");
    //     rtc.adjust(DateTime(2025, 4, 5, 13, 42, 0));
    // }

    // Initialize MicroSD card
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("Failed to initialize MicroSD card!");
        while (1);
    }

    // Initialize color palette for display
    initColors(display);

    // Create FreeRTOS tasks with appropriate priorities and core assignment
    xTaskCreatePinnedToCore(readSensorTask, "ReadSensor", 2048, NULL, 2, &readSensorTaskHandle, 1);     // High priority
    xTaskCreatePinnedToCore(printDataTask, "PrintData", 2048, NULL, 1, &printDataTaskHandle, 1);        // Medium priority
    xTaskCreatePinnedToCore(saveDataTask, "SaveData", 4096, NULL, 1, &saveDataTaskHandle, 0);           // Low priority
    xTaskCreatePinnedToCore(printPanel, "PrintPanelTask", 6411, NULL, 3, &printPanelTaskHandle, 0);     // Highest priority
    xTaskCreatePinnedToCore(alarmTask, "AlarmTask", 2048, NULL, 1, NULL, 1);                            // Medium priority
}


void loop() {
    // Nothing to do here, FreeRTOS handles all tasks
    vTaskDelay(pdMS_TO_TICKS(3000));     // Add a small delay to prevent watchdog timer reset
}
