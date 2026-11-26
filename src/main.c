// Petteri Keskikuru
// Aaro Sirviö
// Sampo Suhonen
// Pico W number 0431
// Bonus points: Aaro Sirviö 3, Petteri Keskikuru 2, Sampo Suhonen 1

// What everyone was responsible for:
// Petteri Keskikuru: gyrotask
// Aaro Sirviö: morsetask
// Sampo Suhonen usbtask

#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <task.h>

#include <math.h>               //for fabsf()
#include <stdbool.h>            //for symbolready flag

#include "tkjhat/sdk.h"

/*
#include "usbSerialDebug/helper.h"
#include "tusb.h"
#include "class/cdc/cdc_device.h"
*/

// Default stack size for the tasks. It can be reduced to 1024 if task is not using lot of memory.
#define DEFAULT_STACK_SIZE 2048 

//#define CDC_ITF_TX 1          //Defining CDC interface index for morse data transfer

#define LOOP_PERIOD 20          //morseTask loop period
#define VALID_ROTATE_LOOPS 25      //how many loops required for a valid dot/dash symbol
#define VALID_SPACE_LOOPS 50    //how many loops required in default position for 1 space
#define VALID_ROTATE 0.75f         //gyro_accel value threshold for a valid dot/dash      AI used to help adjust value
#define VALID_DEFAULT 0.2f     //max variance gyro_accel value for a default position     AI used to help adjust value

volatile float gyro_accel[3];      // gyroscope acceleration values of x, y, z
volatile char symbol = ' ';     // global variable for symbol to be sent to usbTask
volatile bool symbolready = false;  //flag for if next symbol is ready to be sent


static void gyro_task(void *pvParameters)     // create gyro task and define it for main() to use
{
    (void)pvParameters;   // values as listed in ICM42670_read_sensor_data() in TKJHAT sdk.c
    float ax, ay, az;   // acceleration values
    float gx, gy, gz;   // gyro values in degrees per second
    float t;            // temperature (listed in ICM42670_read_sensor_data() function but not used)

    if (init_ICM42670() != 0) {             // AI told these 2 if statements are a good step for testing and troubleshooting
        printf("Initialization failed.\n");   // if init fails, print error message to terminal
        vTaskDelete(NULL);                    // and delete task to prevent it from running without sensor
    }
    if (ICM42670_start_with_default_values() != 0) {
        printf("Failed to start.\n");        // if start with default values fails, print error and delete task
        vTaskDelete(NULL);
    }

    TickType_t tickTime = xTaskGetTickCount();    // initialize tick time for timing in this loop

    for (;;) {      // infinite loop for the gyro task
        if (ICM42670_read_sensor_data(  &ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {         // reads sensor data, if it exists store it in a global variable
            gyro_accel[0] = ax;    // first value of stored gyro values global variable
            gyro_accel[1] = ay;    // 2nd value of gyro
            gyro_accel[2] = az;    // 3rd value of gyro


        } else {                                        // if reading sensor data fails, do this      AI said this is good for troubleshooting and testing
            printf("Failed to read sensor data.\n");    // prints error message to terminal
        }

        vTaskDelayUntil(&tickTime,pdMS_TO_TICKS(LOOP_PERIOD)    // wait for LOOP_PERIOD amount of time before running loop again, ticktime used to know timings
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
        //if(symbolready) continue;   //if a symbol is ready, move to usbTask     AI was used to help with loop logic and taskdelay placement

        float x = gyro_accel[0];       //the first value of gyro_accel, interpreted as x-axis value
        float y = gyro_accel[1];       //the second value of gyro_accel, corresponding to y-axis
        char corresponding = 0;     //corresponding symbol to the gyro_accel values

        //the thresholds are measured using fabsf(), so both negative and positive values are taken into account
        if(fabsf(x) < VALID_DEFAULT && fabsf(y) < VALID_DEFAULT) corresponding = ' ';   //if both x and y are within the range of default pos, the symbol is space
        else if(y > VALID_ROTATE && fabsf(x) < VALID_DEFAULT) corresponding = '.';        //if device is rotated in y-axis, the symbol is dot
        else if(x > VALID_ROTATE && fabsf(y) < VALID_DEFAULT) corresponding = '-';        //if device is rotated in x-axis, the symbol is dash

        //if(corresponding == 0) continue;    //skip this loop, if no valid position is found     AI was used to help with loop logic

        if(corresponding == symbolpos) loopcount++; //if the current loop's position is the same as the previous loop's, add to loopcount
        else {
            symbolpos = corresponding;      //set the current position as the previous position for the next loop
            loopcount = 1;                  //begin the counter
        }

        if(corresponding != ' ' && loopcount == VALID_ROTATE_LOOPS) {                    //if non-default position is measured for enough time
            symbol = corresponding;     //set the current symbol as the output symbol for usbTask
            symbolready = true;         //a new symbol is ready to be sent to usbTask, so next loop is skipped
            spacecount = 2;             //a maximum of two spaces can be sent after a dot/dash
            printf("%c", symbol);
        }
        //if there are spaces left to be sent, and default position is measured for at least one or two space loops   AI was used to help with deciding if
        //spaces can be sent and how many
        else if(corresponding == ' ' && spacecount > 0 && (loopcount == VALID_SPACE_LOOPS || loopcount == 2 * VALID_SPACE_LOOPS)) {
            symbol = ' ';
            symbolready = true;
            spacecount--;               //reduce the amount of spaces that can be sent by one
            printf("%c", symbol);
        }
        //if enough time passes after sending two spaces, the message ends and a newline is sent
        else if(corresponding == ' ' && spacecount == 0 && loopcount == (4 * VALID_SPACE_LOOPS)) {
            printf("\n");
        }
    }
}

/*
static void send_cdc1_bytes(const uint8_t *data, size_t len) {          //Function for data writing to TinyUSB CDC interface (AI was used to make the foundation)
    if (len == 0) {         //Check for no data
        return;
    }

    size_t written = 0;         //Track how many bytes have been written
    while (written < len) {     //Loop until needed bytes are written
        tud_task();             //Service for TinyUSB stack for transfer
        if (tud_cdc_n_ready(CDC_ITF_TX)) {          //Check if CDC is ready
            uint32_t n = tud_cdc_n_write(CDC_ITF_TX, data + written, (uint32_t)(len - written));            //Writing remaining bytes to CDC buffer
            written += n;           //Increase the written data
            tud_cdc_n_write_flush(CDC_ITF_TX);          //Flush the buffer to CDC
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));       //Wait to run loop again
        }
    }
}

//USB task (AI was used to make the foundation)
static void usbTask(void *arg) {
    (void)arg;

    char current_symbol = 0;            //Copy of current symbol
    bool has_new_symbol = false;        //Indicator for new symbol
    int consecutive_spaces = 0;         //Counter for consecutive spaces sent
    bool message_terminated = false;    //Tracker if newline has been sent

    for (;;) {          //Loop for USB task
        tud_task();

        has_new_symbol = false;         //Reset new symbol flag

        if (symbolready) {          //Check from morseTask wheter a new symbol was made
            current_symbol = symbol;            //Copy symbol
            symbolready = false;                //Clear after getting a symbol
            has_new_symbol = true;              //Mark for symbol being ready
        }

        if (has_new_symbol) {
            send_cdc1_bytes((const uint8_t *)&current_symbol, 1);           //Transfer the symbol over CDC 

            if (current_symbol == ' ') {            //Check for empty space
                consecutive_spaces++;               //Increase the consecutive_spaces counter
                if (consecutive_spaces >= 2 && !message_terminated) {           //After two consecutive spaces have been sent
                    const char end_seq[] = "\n";            //Add newline to complete terminator
                    send_cdc1_bytes((const uint8_t *)end_seq, 1);           //Transfer newline
                    message_terminated = true;          //Mark task as complete
                }
            } else {
                consecutive_spaces = 0;         //Reset the consecutive_spaces counter
                message_terminated = false;         //Reset the task
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD));         //Delay for LOOP_PERIOD before restarting
        }
    }
}
*/

int main() {
    stdio_init_all();

    init_hat_sdk();
    sleep_ms(300); //Wait some time so initialization of USB and hat is done.

    TaskHandle_t myGyroTask = NULL;

    BaseType_t result = xTaskCreate(
        gyro_task,
        "GyroTask",
        DEFAULT_STACK_SIZE,
        NULL,
        2,
        &myGyroTask
    );

    if (result != pdPASS) {
        printf("Gyro Task creation failed\n");
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
    
    /*
    TaskHandle_t hUSB = NULL;

    BaseType_t usb_res = xTaskCreate(
        usbTask,
        "usb",
        DEFAULT_STACK_SIZE,
        NULL,
        3,
        &hUSB
    );
    if (usb_res != pdPASS) {
        printf("USB Task creation failed\n");
        return 0;
    }

#if (configNUMBER_OF_CORES > 1)
    vTaskCoreAffinitySet(hUSB, 1u << 0);
#endif

    if (!usb_serial_init()) {
        return 0;
    }
    */

    // Start the scheduler (never returns)
    vTaskStartScheduler();

    // Never reach this line.
    return 0;
}

