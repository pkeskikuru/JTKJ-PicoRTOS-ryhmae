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

//#include "usbSerialDebug/helper.h"
//#include "tusb.h"
//#include "class/cdc/cdc_device.h"

// Default stack size for the tasks. It can be reduced to 1024 if task is not using lot of memory.
#define DEFAULT_STACK_SIZE 2048 

//#define CDC_ITF_TX 1

#define LOOP_PERIOD 20          //morseTask loop period
#define VALID_ROTATE_LOOPS 5      //how many loops required for a valid dot/dash symbol
#define VALID_SPACE_LOOPS 50    //how many loops required in default position for 1 space
#define VALID_ROTATE 0.75f         //gyro_accel value threshold for a valid dot/dash      AI used to help adjust value
#define VALID_DEFAULT 0.2f     //max variance gyro_accel value for a default position     AI used to help adjust value

volatile float gyro_accel[3];      // gyroscope values of x, y, z in degrees per second
volatile char symbol = ' ';     // global variable for symbol to be sent to usbTask
volatile bool symbolready = false;  //flag for if next symbol is ready to be sent


static void gyro_task(void *pvParameters)     // create gyro task
{
    (void)pvParameters;   // values as listed in ICM42670_read_sensor_data() in TKJHAT sdk.c
    float ax, ay, az;   // acceleration values
    float gx, gy, gz;   // gyro values
    float t;            // temperature (listed in ICM42670_read_sensor_data() function but not used)

    if (init_ICM42670() != 0) {             // initialize gyro sensor
        printf("Initialization failed.\n");   // if init fails, print error and delete task
        vTaskDelete(NULL);
    }
    if (ICM42670_start_with_default_values() != 0) {
        printf("Failed to start.\n");        // if start with default values fails, print error and delete task
        vTaskDelete(NULL);
    }

    TickType_t tickTime = xTaskGetTickCount();    // initialize tick time for later use

    for (;;) {      // infinite loop for the gyro task until the task is deleted or the program ends
        if (ICM42670_read_sensor_data(  &ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {         // read sensor data, if it exists store it in a global variable
            gyro_accel[0] = ax;
            gyro_accel[1] = ay;
            gyro_accel[2] = az;


        } else {
            printf("Failed to read sensor data.\n");
        }

        vTaskDelayUntil(&tickTime,pdMS_TO_TICKS(LOOP_PERIOD)
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
// Lähettää tavut TinyUSB CDC 1 -kanavalle (CDC_ITF_TX = 1)
static void send_cdc1_bytes(const uint8_t *data, size_t len) {
    if (len == 0) {
        return;
    }

    size_t written = 0;
    while (written < len) {
        tud_task();
        if (tud_cdc_n_ready(CDC_ITF_TX)) {
            uint32_t n = tud_cdc_n_write(CDC_ITF_TX, data + written, (uint32_t)(len - written));
            written += n;
            tud_cdc_n_write_flush(CDC_ITF_TX);
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

// USB-task: ajaa tud_task()-kutsun ja lähettää Morse-merkit ("  \n"-päätteellä) CDC 1:een
static void usbTask(void *arg) {
    (void)arg;

    char current_symbol = 0;
    bool has_new_symbol = false;
    int consecutive_spaces = 0;
    bool message_terminated = false;

    for (;;) {
        tud_task();

        has_new_symbol = false;

        if (symbolready) {
            current_symbol = symbol;
            symbolready = false;
            has_new_symbol = true;
        }

        if (has_new_symbol) {
            send_cdc1_bytes((const uint8_t *)&current_symbol, 1);

            if (current_symbol == ' ') {
                consecutive_spaces++;
                if (consecutive_spaces >= 2 && !message_terminated) {
                    const char end_seq[] = "\n";
                    send_cdc1_bytes((const uint8_t *)end_seq, 1);
                    message_terminated = true;
                }
            } else {
                consecutive_spaces = 0;
                message_terminated = false;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD));
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

    // Alusta TinyUSB ja usb-serial-debug (CDC0) ennen schedulerin käynnistystä
    tusb_init();
    if (!usb_serial_init()) {
        return 0;
    }
*/
    // Start the scheduler (never returns)
    vTaskStartScheduler();

    // Never reach this line.
    return 0;
}

