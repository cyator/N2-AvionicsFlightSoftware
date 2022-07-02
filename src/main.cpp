#include <Arduino.h>
#include "../include/checkState.h"
#include "../include/logdata.h"
#include "../include/readsensors.h"
#include "../include/transmitwifi.h"
#include "../include/defs.h"
#include "../include/kalmanfilter.h"

TimerHandle_t ejectionTimerHandle = NULL;

portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;

TaskHandle_t WiFiTelemetryTaskHandle;

TaskHandle_t GetDataTaskHandle;

// if 1 chute has been deployed
uint8_t isChuteDeployed = 0;

float BASE_ALTITUDE = 0;

float previousAltitude;

volatile int state = 0;

static uint16_t wifi_queue_length = 100;

static QueueHandle_t wifi_telemetry_queue;

// callback for done ejection
void ejectionTimerCallback(TimerHandle_t ejectionTimerHandle)
{
    digitalWrite(EJECTION_PIN, LOW);
    isChuteDeployed = 1;
}

// Ejection fires the explosive charge using a relay or a mosfet
void ejection()
{
    if (isChuteDeployed == 0)
    {
        digitalWrite(EJECTION_PIN, HIGH);
        // TODO: is 3 seconds enough?
        ejectionTimerHandle = xTimerCreate("EjectionTimer", 3000 / portTICK_PERIOD_MS, pdFALSE, (void *)0, ejectionTimerCallback);
        xTimerStart(ejectionTimerHandle, portMAX_DELAY);
    }
}

LogData readData()
{
    LogData ld = {0};
    SensorReadings readings = {0};
    FilteredValues filtered_values = {0};

    readings = get_readings();

    // TODO: very important to know the orientation of the altimeter
    filtered_values = kalmanUpdate(readings.altitude, readings.ay);

    // using mutex to modify state
    portENTER_CRITICAL(&mutex);
    state = checkState(filtered_values.displacement, previousAltitude, filtered_values.velocity, filtered_values.acceleration, state);
    portEXIT_CRITICAL(&mutex);
    previousAltitude = filtered_values.displacement;

    ld = formart_SD_data(readings, filtered_values);
    ld.state = state;
    ld.timeStamp = millis();

    return ld;
}

/*
**********Time Taken for each Task******************
        Get Data Task  - 36ms
        WiFiTelemetryTask -74ms

*/

void GetDataTask(void *parameter)
{

    LogData ld = {0};

    static int droppedWiFiPackets = 0;

    for (;;)
    {

        ld = readData();

        if (xQueueSend(wifi_telemetry_queue, (void *)&ld, 0) != pdTRUE)
        {
            droppedWiFiPackets++;
        }

        debugf("Dropped WiFi Packets : %d\n", droppedWiFiPackets);

        // yield to WiFi Telemetry task
        //  vTaskDelay(36 / portTICK_PERIOD_MS);
    }
}

void WiFiTelemetryTask(void *parameter)
{
    struct LogData ld = {0};

    for (;;)
    {

        xQueueReceive(wifi_telemetry_queue, (void *)&ld, 10);

        handleWiFi(ld);

        // yield to Get Data task
        // vTaskDelay(74 / portTICK_PERIOD_MS);
    }
}

void setup()
{

    Serial.begin(BAUD_RATE);
    // delay to allow serial monitor to connect
   // vTaskDelay(1000 / portTICK_PERIOD_MS);
    // set up ejection pin
    pinMode(EJECTION_PIN, OUTPUT);

    setup_wifi();

    init_sensors();

    // get the base_altitude
    BASE_ALTITUDE = get_base_altitude();

    wifi_telemetry_queue = xQueueCreate(wifi_queue_length, sizeof(SendValues));

    // initialize core tasks
    xTaskCreatePinnedToCore(GetDataTask, "GetDataTask", 3000, NULL, 1, &GetDataTaskHandle, 0);
    xTaskCreatePinnedToCore(WiFiTelemetryTask, "WiFiTelemetryTask", 4000, NULL, 1, &WiFiTelemetryTaskHandle, 1);
}
void loop()
{
}