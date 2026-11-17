
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include <math.h>               //for fabsf()
#include <stdbool.h>            //for symbolready flag

#include "tkjhat/sdk.h"

// Default stack size for the tasks. It can be reduced to 1024 if task is not using lot of memory.
#define DEFAULT_STACK_SIZE 2048 

#define LOOP_PERIOD 20          //morseTask loop period
#define VALID_ROTATE_LOOPS 5      //how many loops required for a valid dot/dash symbol
#define VALID_SPACE_LOOPS 50    //how many loops required in default position for 1 space
#define VALID_ROTATE 0.5f         //accel_g value threshold for a valid dot/dash          AI?
#define VALID_DEFAULT 0.25f     //max variance accel_g value for a default position     AI?

volatile float accel_g[3];      //accelerometer values of x, y, z; converted to g units
volatile char symbol = ' ';     //global variable for symbol to be sent to usbTask
volatile bool symbolready = false;  //flag for if next symbol is ready to be sent

//Add here necessary states
enum state { IDLE=1 };
enum state programState = IDLE;

static void example_task(void *arg){
    (void)arg;

    for(;;){
        tight_loop_contents(); // Modify with application code here.
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void imu_task(void *pvParameters)
{
    (void)pvParameters;

    float ax, ay, az;
    float gx, gy, gz;
    float temperature;

    // Initialize IMU sensor
    if (init_ICM42670() != 0) {
        printf("Failed to initialize ICM-42670P.\n");
        vTaskDelete(NULL);
    }

    if (ICM42670_start_with_default_values() != 0) {
        printf("Failed to start ICM-42670P.\n");
        vTaskDelete(NULL);
    }

    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;) {

        // Read sensor data
        if (ICM42670_read_sensor_data(
                &ax, &ay, &az,
                &gx, &gy, &gz,
                &temperature) == 0) {

            // Store acceleration values
            taskENTER_CRITICAL();

            accel_g[0] = ax;
            accel_g[1] = ay;
            accel_g[2] = az;

            taskEXIT_CRITICAL();
        }

        // Wait until the next 20 ms measurement interval
        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(LOOP_PERIOD)
        );
    }
}

//morseTask
static void morse_task(void *arg){
    (void)arg;

    char symbolpos = 0;         //what symbol the previous loop's position represents
    int loopcount = 0;          //how many loops the device has been in the most recent symbolpos
    int spacecount = 0;         //how many spaces can be sent

    for(;;){
        
        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD));
        if(symbolready) continue;   //if a symbol is ready, move to usbTask     AI?

        float x = accel_g[0];       //the first value of accel_g, interpreted as x-axis value
        float y = accel_g[1];       //the second value of accel_g, corresponding to y-axis
        char corresponding = 0;     //corresponding symbol to the accel_g values

        //the thresholds are measured using fabsf(), so both negative and positive values are taken into account
        if(fabsf(x) < VALID_DEFAULT && fabsf(y) < VALID_DEFAULT) corresponding = ' ';   //if both x and y are within the range of default pos, the symbol is space
        else if(y > VALID_ROTATE && fabsf(x) < VALID_DEFAULT) corresponding = '.';        //if device is rotated in y-axis, the symbol is dot
        else if(x > VALID_ROTATE && fabsf(y) < VALID_DEFAULT) corresponding = '-';        //if device is rotated in x-axis, the symbol is dash

        if(corresponding == 0) continue;    //skip this loop, if no valid position is found     AI?

        if(corresponding == symbolpos) loopcount++; //if the current loop's position is the same as the previous loop's, add to loopcount
        else {
            symbolpos = corresponding;      //set the current position as the previous position for the next loop
            loopcount = 1;                  //begin the counter
        }

        if(corresponding != ' ' && loopcount == VALID_ROTATE_LOOPS) {                    //if non-default position is measured for enough time
            symbol = corresponding;     //set the current symbol as the output symbol for usbTask
            symbolready = true;         //a new symbol is ready to be sent to usbTask, so next loop is skipped
            spacecount = 2;             //a maximum of two spaces can be sent after a dot/dash
        }
        //if there are spaces left to be sent, and default position is measured for at least one or two space loops   AI?
        else if(corresponding == ' ' && spacecount > 0 && loopcount == VALID_SPACE_LOOPS || loopcount == 2 * VALID_SPACE_LOOPS) {
            symbol = ' ';
            symbolready = true;
            spacecount--;               //reduce the amount of spaces that can be sent by one
        }
    }
}

int main() {
    stdio_init_all();
    // Uncomment this lines if you want to wait till the serial monitor is connected
    /*while (!stdio_usb_connected()){
        sleep_ms(10);
    }*/ 
    init_hat_sdk();
    sleep_ms(300); //Wait some time so initialization of USB and hat is done.

     TaskHandle_t myIMUTask = NULL;

    BaseType_t result = xTaskCreate(
        imu_task,
        "IMUTask",
        DEFAULT_STACK_SIZE,
        NULL,
        2,
        &myIMUTask
    );

    if (result != pdPASS) {
        printf("IMU Task creation failed\n");
        return 0;
    }

    TaskHandle_t morseHandle = NULL;

    BaseType_t morse = xTaskCreate(morse_task,
                "morse",
                DEFAULT_STACK_SIZE,
                NULL,
                2,
                &morseHandle);

    if(morse != pdPASS) {
        printf("Morse Task creation failed\n");
        return 0;
    }
    
    // Start the scheduler (never returns)
    vTaskStartScheduler();

    // Never reach this line.
    return 0;
}

