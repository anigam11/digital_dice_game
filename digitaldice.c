#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_fxos.h"
#include "pin_mux.h"
#include "fsl_gpio.h"
#include "stdlib.h"
#include "time.h"

#define DICE_THRESHOLD 2000U
#define RED_LED_GPIO GPIOC
#define RED_LED_PIN 9
#define GREEN_LED_GPIO PORTE
#define GREEN_LED_PIN 6
#define BUTTON_GPIO GPIOA
#define BUTTON_PIN 4

volatile int16_t xAccel = 0;
volatile int16_t yAccel = 0;
volatile int16_t zAccel = 0;

/* FXOS device address */
const uint8_t g_accel_address[] = {0x1CU, 0x1DU, 0x1EU, 0x1FU};

void initLED()
{
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput, 0,
    };
    GPIO_PinInit(RED_LED_GPIO, RED_LED_PIN, &led_config);

    gpio_pin_config_t greenLedConfig = {
           kGPIO_DigitalOutput, 0,
       };
       GPIO_PinInit(GREEN_LED_GPIO, GREEN_LED_PIN, &greenLedConfig);

}

void ToggleGreenLED(void);
void ToggleRedLED(void);

void initButton()
{
    gpio_pin_config_t button_config = {
        kGPIO_DigitalInput, 0,
    };
    GPIO_PinInit(BUTTON_GPIO, BUTTON_PIN, &button_config);
}

/* Function to simulate the roll of a digital dice */
static void RollDigitalDice(void)
{
    PRINTF("Rolling the digital dice...\r\n");

    srand((unsigned)time(NULL));
    int diceResult = rand() % 6 + 1;

    asm volatile(
           "mov r0, %[diceResult]\n"    // Load diceResult into register r0
           "loop:\n"
           "    ldr r1, =1000\n"       // Load delay value into register r1
           "delay:\n"
           "    subs r1, #1\n"            // Decrement the delay counter
           "    bne delay\n"              // Branch back if not zero
           "    str r0, %[ledPort]\n"     // Store diceResult to the LED port
           "    ldr r1, =1000\n"        // Load delay value into register r1
           "delay_off:\n"
           "    subs r1, #1\n"            // Decrement the delay counter
           "    bne delay_off\n"          // Branch back if not zero
           "    str r0, %[ledPortClear]\n" // Clear the LED port
           "    subs %[diceResult], #1\n" // Decrement diceResult
           "    bne loop\n"               // Branch back if not zero
           :
           : [ledPort] "m"(RED_LED_GPIO->PDOR), [ledPortClear] "m"(RED_LED_GPIO->PCOR), [diceResult] "r"(diceResult)
           : "r0", "r1", "cc");

       PRINTF("Dice result: %d\r\n", diceResult);
}

void BlinkGreenLED(void)
{
    asm volatile(
        "mov r0, #10\n"                  // Load loop counter
        "green_loop:\n"
        "    ldr r1, =5000\n"            // Load delay value into register r1
        "green_delay_on:\n"
        "    subs r1, #1\n"               // Decrement the delay counter
        "    bne green_delay_on\n"        // Branch back if not zero
        "    bl ToggleGreenLED\n"         // Call C function to toggle the LED port
        "    ldr r1, =5000\n"            // Load delay value into register r1
        "green_delay_off:\n"
        "    subs r1, #1\n"               // Decrement the delay counter
        "    bne green_delay_off\n"       // Branch back if not zero
        "    subs r0, #1\n"               // Decrement loop counter
        "    bne green_loop\n"            // Branch back if not zero
        :
        :
        : "r0", "r1", "cc");
}

void ToggleGreenLED(void)
{
    // Toggle the green LED
    GPIO_PortToggle(GREEN_LED_GPIO, 1U << GREEN_LED_PIN);
}

void ToggleRedLED(void)
{
    // Toggle the red LED
    GPIO_PortToggle(RED_LED_GPIO, 1U << RED_LED_PIN);
}

void DelayMicroseconds(uint32_t us)
{
    asm volatile(
        "1: subs %[us], %[us], #1\n"
        "   bne 1b"
        : [us] "+r"(us)
        :
        : "cc");
}

int main(void)
{
    fxos_handle_t fxosHandle = {0};
    fxos_data_t sensorData = {0};
    fxos_config_t config = {0};
    uint8_t array_addr_size = 0;
    status_t result = kStatus_Fail;
    bool buttonPressed = false;

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_InitDebugConsole();

    initLED();
    initButton();

    config.I2C_SendFunc = BOARD_Accel_I2C_Send;
    config.I2C_ReceiveFunc = BOARD_Accel_I2C_Receive;

    array_addr_size = sizeof(g_accel_address) / sizeof(g_accel_address[0]);
    for (uint8_t i = 0; i < array_addr_size; i++)
    {
        config.slaveAddress = g_accel_address[i];
        result = FXOS_Init(&fxosHandle, &config);
        if (result == kStatus_Success)
        {
            break;
        }
    }

    if (result != kStatus_Success)
    {
        PRINTF("\r\nSensor device initialize failed!\r\n");
        return -1;
    }

    PRINTF("Welcome to the Digital Dice game!\r\n");

    while (1)
    {
        if (FXOS_ReadSensorData(&fxosHandle, &sensorData) != kStatus_Success)
        {
            return -1;
        }

        xAccel = (int16_t)((uint16_t)((uint16_t)sensorData.accelXMSB << 8) | (uint16_t)sensorData.accelXLSB);
        yAccel = (int16_t)((uint16_t)((uint16_t)sensorData.accelYMSB << 8) | (uint16_t)sensorData.accelYLSB);
        zAccel = (int16_t)((uint16_t)((uint16_t)sensorData.accelZMSB << 8) | (uint16_t)sensorData.accelZLSB);

        if ((abs(xAccel) > DICE_THRESHOLD) || (abs(yAccel) > DICE_THRESHOLD) || (abs(zAccel) > DICE_THRESHOLD))
        {
            if (!buttonPressed)
            {
                RollDigitalDice();
                buttonPressed = true;
            }
            for (volatile int i = 0; i < 1000; ++i)
            {
            }
        }
        else
        {
            buttonPressed = false;
        }
    }
}
