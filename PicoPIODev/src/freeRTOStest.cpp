/* Scheduler include files. */
#include "FreeRTOS.h"

/* Library includes. */
#include "task.h"   /* RTOS task related API prototypes. */
#include "queue.h"  /* RTOS queue related API prototypes. */
#include "timers.h" /* Software timer related API prototypes. */
#include "semphr.h" /* Semaphore related API prototypes. */
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include <Arduino.h>

/* Priorities at which the tasks are created. */
#define RADIO_HB_TASK_PRIORITY (tskIDLE_PRIORITY + 4)
#define RADIO_TELEM_TASK_PRIORITY (tskIDLE_PRIORITY + 4)
#define SENSOR_READ_TO_TELEM_PRIORITY (tskIDLE_PRIORITY + 3)
#define SENSOR_READER_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define SENSOR_INTERRUPT_TASK_PRIORITY (tskIDLE_PRIORITY + 1)

/* The rate at which data is sent to the queue.  The 200ms value is converted
to ticks using the portTICK_PERIOD_MS constant. */
#define HB_SEND_FREQUENCY_MS (250 / portTICK_PERIOD_MS)
#define TELEM_SEND_FREQUENCY_MS (1000 / portTICK_PERIOD_MS)
#define INTERRUPT_GEN_FREQUENCY_MS (100 / portTICK_PERIOD_MS)

/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define DATA_QUEUE_LENGTH (128)

/* The LED toggled by the Rx task. */
#define mainTASK_LED (PICO_DEFAULT_LED_PIN)

/* Prototypes for the standard FreeRTOS callback/hook functions implemented
within this file. */
void vApplicationMallocFailedHook(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName);
void vApplicationTickHook(void);

/*-----------------------------------------------------------*/

void setup(void);
void loop(void);

/*
 * Tasks that are run
 */

// Task to simulate sensor interrupts
static void sensorInterruptTimerTask(TimerHandle_t xTimer);

// Actual sensor reader tasks to read raw data
static void gyroReaderTask(void *pvParameters);
static void accelReaderTask(void *pvParameters);
static void magReaderTask(void *pvParameters);

// Read sensor data to telemetry struct
static void sensorReadToTelemTask(void *pvParameters);

// Radio tasks
static void radioHeartbeatTask(void *pvParameters);
static void radioTelemetryTask(void *pvParameters);

// /* Create the queue. */
//     xQueue = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));

//     if (xQueue != NULL)
//     {
//         /* Start the two tasks as described in the comments at the top of this
//         file. */
//         xTaskCreate(prvQueueReceiveTask,             /* The function that implements the task. */
//                     "Rx",                            /* The text name assigned to the task - for debug only as it is not used by the kernel. */
//                     configMINIMAL_STACK_SIZE,        /* The size of the stack to allocate to the task. */
//                     NULL,                            /* The parameter passed to the task - not used in this case. */
//                     mainQUEUE_RECEIVE_TASK_PRIORITY, /* The priority assigned to the task. */
//                     NULL);                           /* The task handle is not required, so NULL is passed. */

//         xTaskCreate(prvQueueSendTask, "TX", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL);
//     }

/*-----------------------------------------------------------*/

// Sensor type enum: what kind of sensor is sending the data?
typedef enum
{
    GYRO = 1,
    MAG = 2,
    ACC = 3,
    TEMP = 4
} SensorType_t;

// Orientable enum: where is the data coming from?
typedef enum
{
    NA = 0,
    X = 1,
    Y = 2,
    Z = 3,
    ROLL = 4,
    PITCH = 5,
    YAW = 6,
    N = 7,
    E = 8,
    D = 9,
    LAT = 10,
    LNG = 11
} Orientable_t;

// Sensor data struct which is added to queue
struct SensorData_s
{
    double data;
    uint32_t timestamp;
    Orientable_t sensorAxis;
    uint32_t crc; // TODO implement this
};

// Queueable sensor data
struct Queueable_s
{
    SensorData_s data;
    SensorType_t type;
};

// Sensor itself
struct Sensor_s
{
    SensorType_t type;
    const char *name;
    SensorData_s *data;
    int dataLen;
    SemaphoreHandle_t sem;
};

// Telemetry object of multiple sensors
struct Telemetry_s
{
    Sensor_s gyro;
    Sensor_s acc;
    Sensor_s mag;
};
struct Telemetry_s telem;

QueueHandle_t sensorDataQueue = NULL;

/*-----------------------------------------------------------*/

void setup(void)
{
    Serial.begin(9600);
    while (!Serial)
        ;
    delay(3000);
    Serial.println(" Starting RTOS test");

    pinMode(mainTASK_LED, OUTPUT);

    const char *rtos_name;
#if (portSUPPORT_SMP == 1)
    rtos_name = "FreeRTOS SMP";
#else
    rtos_name = "FreeRTOS";
#endif

#if (portSUPPORT_SMP == 1) && (configNUM_CORES == 2)
    Serial.print(rtos_name);
    Serial.println(" running on both cores");
#else
#error Only works on both cores
#endif

    xTimerCreate(
        "InterruptGen",
        INTERRUPT_GEN_FREQUENCY_MS,
        pdTRUE,
        (void *)0,
        sensorInterruptTimerTask);

    xTaskCreate(
        gyroReaderTask,
        "GyroReader",
        configMINIMAL_STACK_SIZE, NULL, SENSOR_READER_TASK_PRIORITY, NULL);
    xTaskCreate(
        accelReaderTask,
        "AccelReader",
        configMINIMAL_STACK_SIZE, NULL, SENSOR_READER_TASK_PRIORITY, NULL);
    xTaskCreate(
        magReaderTask,
        "MagReader",
        configMINIMAL_STACK_SIZE, NULL, SENSOR_READER_TASK_PRIORITY, NULL);

    xTaskCreate(
        sensorReadToTelemTask,
        "ReadQueueToTelem",
        configMINIMAL_STACK_SIZE, NULL, SENSOR_READ_TO_TELEM_PRIORITY, NULL);

    xTaskCreate(
        radioHeartbeatTask,
        "RadioHeartbeat",
        configMINIMAL_STACK_SIZE, NULL, RADIO_HB_TASK_PRIORITY, NULL);
    xTaskCreate(
        radioTelemetryTask,
        "RadioTelem",
        configMINIMAL_STACK_SIZE, NULL, RADIO_TELEM_TASK_PRIORITY, NULL);

    Serial.println("Initial tasks created & running");
}
void loop(void) {}
/*-----------------------------------------------------------*/

// Sensor readers first

static void gyroReaderTask(void *pvParameters)
{
    // Perform initial configuration of gyro
    Sensor_s gyro = telem.gyro;
    gyro.sem = xSemaphoreCreateBinary();
    gyro.type = SensorType_t::GYRO;
    gyro.name = "Gyroscope BMI088";
    gyro.dataLen = 0;
    gyro.data = (*SensorData_s)[];

    // Data reader handler
    for (;;)
    {
        // Attempt to take semaphore, if this is possible then we want to read all items into sensor queue
        if (sensorDataQueue != NULL && xSemaphoreTake(gyro.sem, portMAX_DELAY))
        {
            vTaskEnterCritical();
            for (int i = 0; i < gyro.dataLen; i++)
            {
                xQueueSend(sensorDataQueue, &gyro.data[i], 0U);
            }
            gyro.data = (*SensorData_s)[];
            vTaskExitCritical();
        }
    }
}

static void sensorInterruptTimerTask(TimerHandle_t xTimer)
{
    static volatile int sensorIdx = 0;
    for (;;)
    {
        telem.gyro.sem
    }
}

static void prvQueueSendTask(void *pvParameters)
{
    TickType_t xNextWakeTime;
    const unsigned long ulValueToSend = 100UL;

    /* Initialise xNextWakeTime - this only needs to be done once. */
    xNextWakeTime = xTaskGetTickCount();

    for (;;)
    {
        /* Place this task in the blocked state until it is time to run again. */
        vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS);

        /* Send to the queue - causing the queue receive task to unblock and
        toggle the LED.  0 is used as the block time so the sending operation
        will not block - it shouldn't need to block as the queue should always
        be empty at this point in the code. */
        xQueueSend(xQueue, &ulValueToSend, 0U);
    }
}
/*-----------------------------------------------------------*/

static void prvQueueReceiveTask(void *pvParameters)
{
    unsigned long ulReceivedValue;
    const unsigned long ulExpectedValue = 100UL;

    for (;;)
    {
        /* Wait until something arrives in the queue - this task will block
        indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
        FreeRTOSConfig.h. */
        xQueueReceive(xQueue, &ulReceivedValue, 100U);

        /*  To get here something must have been received from the queue, but
        is it the expected value?  If it is, toggle the LED. */
        if (ulReceivedValue == ulExpectedValue)
        {
            digitalWrite(mainTASK_LED, !digitalRead(mainTASK_LED));
            ulReceivedValue = 0U;
        }
    }
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
    /* Called if a call to pvPortMalloc() fails because there is insufficient
    free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    internally by FreeRTOS API functions that create tasks, queues, software
    timers, and semaphores.  The size of the FreeRTOS heap is set by the
    configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

    /* Force an assert. */
    taskENTER_CRITICAL();
    Serial.println("ERROR: malloc failed");
    taskEXIT_CRITICAL();
    configASSERT((volatile void *)NULL);
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void)pcTaskName;
    (void)pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */

    /* Force an assert. */
    taskENTER_CRITICAL();
    Serial.println("ERROR: Stack overflow");
    taskEXIT_CRITICAL();
    configASSERT((volatile void *)NULL);
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
    volatile size_t xFreeHeapSpace;

    /* This is just a trivial example of an idle hook.  It is called on each
    cycle of the idle task.  It must *NOT* attempt to block.  In this case the
    idle task just queries the amount of FreeRTOS heap that remains.  See the
    memory management section on the http://www.FreeRTOS.org web site for memory
    management options.  If there is a lot of heap memory free then the
    configTOTAL_HEAP_SIZE value in FreeRTOSConfig.h can be reduced to free up
    RAM. */
    xFreeHeapSpace = xPortGetFreeHeapSize();

    /* Remove compiler warning about xFreeHeapSpace being set but never used. */
    (void)xFreeHeapSpace;
}

void vApplicationTickHook(void)
{
}