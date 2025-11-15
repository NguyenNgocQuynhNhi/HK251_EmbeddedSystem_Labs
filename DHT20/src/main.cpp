#include <Arduino.h>
#include "DHT20.h"

void TaskLEDControl(void *pvParameters) {
    pinMode(GPIO_NUM_48, OUTPUT); // Initialize LED pin
    int ledState = 0;
    while(1) {
      
      if (ledState == 0) {
          digitalWrite(GPIO_NUM_48, HIGH); // Turn ON LED
          // Serial.println("Led is on.");
      } else {
          digitalWrite(GPIO_NUM_48, LOW); // Turn OFF LED
          // Serial.println("Led is off.");
      }
      ledState = 1 - ledState;
      vTaskDelay(1000);
    }
}

void TaskTemperatue_Humidity(void *pvParameters ) {
    DHT20 dht20;
    Wire.begin(GPIO_NUM_11, GPIO_NUM_12);
    dht20.begin();
    while(1) {
        dht20.read();
        double temperature = dht20.getTemperature();
        double humidity = dht20.getHumidity();

        Serial.print("Temp: ");
        Serial.print(temperature);
        Serial.print(" *C ");

        Serial.print(" Humidity: ");
        Serial.print(humidity);
        Serial.print(" %");

        Serial.println();

        vTaskDelay(5000);
    }
}


void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    xTaskCreate(TaskLEDControl, "LED Control", 2048, NULL, 2, NULL);
    xTaskCreate(TaskTemperatue_Humidity, "Temperature_Humidity", 2048, NULL, 2, NULL);
}

void loop() {
    // Serial.println("Hello Custom Board");
    // delay(1000);
}