/*  Time of Flight for 2DX4 -- Studio W8-0
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c

                The VL53L1X is run with default firmware settings.


            Written by Tom Doyle
            Updated by  Hafez Mousavi Garmaroudi
            Last Update: March 17, 2020

                        Last Update: March 03, 2022
                        Updated by Hafez Mousavi
                        __ the dev address can now be written in its original format.
                                Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file

                        Modified March 16, 2023
                        by T. Doyle
                            - minor modifications made to make compatible with new Keil IDE

*/
#include <stdint.h>

#include <math.h>

#include "PLL.h"

#include "SysTick.h"

#include "uart.h"

#include "onboardLEDs.h"

#include "tm4c1294ncpdt.h"

#include "VL53L1X_api.h"

#define I2C_MCS_ACK 0x00000008    // Data Acknowledge Enable
#define I2C_MCS_DATACK 0x00000008 // Acknowledge Data
#define I2C_MCS_ADRACK 0x00000004 // Acknowledge Address
#define I2C_MCS_STOP 0x00000004   // Generate STOP
#define I2C_MCS_START 0x00000002  // Generate START
#define I2C_MCS_ERROR 0x00000002  // Error
#define I2C_MCS_RUN 0x00000001    // I2C Master Enable
#define I2C_MCS_BUSY 0x00000001   // I2C Busy
#define I2C_MCR_MFE 0x00000010    // I2C Master Function Enable

#define MAXRETRIES 5 // number of receive attempts before giving up

#define CLOCKDEMO 0  //change to 500 for demo

void I2C_Init(void)
{
    SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;   // activate I2C0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1; // activate port B
    while ((SYSCTL_PRGPIO_R & 0x0002) == 0)
    {
    }; // ready?

    GPIO_PORTB_AFSEL_R |= 0x0C; // 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;   // 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C; // 5) enable digital I/O on PB2,3
    //    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

    // 6) configure PB2,3 as I2C
    //  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFF00FF) + 0x00002200; // TED
    I2C0_MCR_R = I2C_MCR_MFE;                                          // 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                   // 8) configure for 10 kbps clock (added 8 clocks of glitch suppression ~50ns)
                                                                       //    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
}

// The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void)
{
    // Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6; // activate clock for Port N
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R6) == 0)
    {
    };                           // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;    // make PG0 in (HiZ)
    GPIO_PORTG_AFSEL_R &= ~0x01; // disable alt funct on PG0
    GPIO_PORTG_DEN_R |= 0x01;    // enable digital I/O on PG0
    // configure PG0 as GPIO
    // GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
    GPIO_PORTG_AMSEL_R &= ~0x01; // disable analog functionality on PN0

    return;
}

// Enable interrupts
void EnableInt(void)
{
    __asm("    cpsie   i\n");
}

// Disable interrupts
void DisableInt(void)
{
    __asm("    cpsid   i\n");
}

// Low power wait
void WaitForInt(void)
{
    __asm("    wfi\n");
}

void PortJ_Init(void)
{
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8; // Activate the clock for Port F
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0)
    {
    };                                // Allow time for clock to stabilize
    GPIO_PORTJ_PCTL_R &= ~0x000000F0; //   Configure PJ1 as GPIO
    GPIO_PORTJ_DIR_R = 0b00000000;    // Enable PJ0 and PJ1 as inputs
    GPIO_PORTJ_AFSEL_R &= ~0x03;      // disable alt funct on Port J pins (PJ1-PJ0)
    GPIO_PORTJ_PUR_R |= 0b00000011;
    GPIO_PORTJ_DEN_R = 0b00000011; // Enable PJ0 and PJ1 as digital pins
    return;
}

void PortH_Init(void)
{
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7; // activate clock for Port H
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0)
    {
    };                           // allow time for clock to stabilize
    GPIO_PORTH_DIR_R |= 0x0F;    // configure Port H pins (PH0-PH3) as output
    GPIO_PORTH_AFSEL_R &= ~0x0F; // disable alt funct on Port M pins (PH0-PH3)
    GPIO_PORTH_DEN_R |= 0x0F;    // enable digital I/O on Port M pins (PH0-PH3)
    // configure Port H as GPIO
    GPIO_PORTH_AMSEL_R &= ~0x0F; // disable analog functionality on Port H	pins (PH0-PH3)
    return;
}



// Increments at least once per button press
volatile unsigned long FallingEdges = 0;
// Interrupt initialization for GPIO Port J IRQ# 51
void PortJ_Interrupt_Init(void)
{
    FallingEdges = 0; // Initialize counter

    GPIO_PORTJ_IS_R &= ~0x03;  // 0 = Edge-sensitive (PJ1, PJ0)
    GPIO_PORTJ_IBE_R &= ~0x03; // 0 = Not both edges (control edge separately)
    GPIO_PORTJ_IEV_R &= ~0x03; // 0 = Falling edge event (for both PJ0, PJ1)
    GPIO_PORTJ_ICR_R = 0x03;   // Clear any prior interrupts (PJ1 and PJ0)
    GPIO_PORTJ_IM_R |= 0x03;   // Enable interrupts on PJ1 and PJ0

    NVIC_EN1_R = 0x00080000;   // Enable interrupt 51 (Port J) in NVIC
    NVIC_PRI12_R = 0xA0000000; // Set interrupt priority to 5

    EnableInt(); // Enable global interrupts
}


void VL53L1X_XSHUT(void)
{
    GPIO_PORTG_DIR_R |= 0x01;        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110; // PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01; // make PG0 input (HiZ)
}

uint16_t dev = 0x29; // address of the ToF sensor as an I2C slave peripheral


int status = 0;

void GPIOJ_IRQHandler(void)
{
    OffLED1(); //Not ready for new interrupt
	
    if (GPIO_PORTJ_MIS_R & 0x01)
    { // Check if PJ0 triggered the interrupt

        if (CLOCKDEMO)
        {
            ClockReadingPN2(CLOCKDEMO); 
        }
				
        else
        {
            UART_printf("999\n");
            FlashLED3(1);
        }
				GPIO_PORTJ_ICR_R = 0x01; // Clear PJ0 interrupt flag
				return;
    }
		
    else
    {

        FallingEdges = FallingEdges + 1; // Increase the global counter variable ;Observe in Debug Watch Window

        GPIO_PORTJ_ICR_R = 0x02; // Acknowledge flag by setting proper bit in ICR register

        uint8_t byteData, sensorState = 0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        uint16_t wordData;
        uint16_t Distance;
        uint16_t SignalRate;
        uint16_t AmbientRate;
        uint16_t SpadNum;
        uint8_t RangeStatus;
        uint8_t dataReady;

        uint32_t delay = 1; // Does your motor spin clockwise or counter-clockwise?
        // int step = 0b00000001; // Starting step pattern

        status = VL53L1X_GetSensorId(dev, &wordData);

        if (FallingEdges == 1)
        {
            // hello world!
            UART_printf("Program Begins\r\n");
            FlashLED3(1);
            int mynumber = 1;
            sprintf(printf_buffer, "2DX ToF Program Studio Code %d\r\n", mynumber);
            UART_printf(printf_buffer);
            FlashLED3(1);
            sprintf(printf_buffer, "(Model_ID, Module_Type)=0x%x\r\n", wordData);
            UART_printf(printf_buffer);
            FlashLED3(1);
        }

        // 1 Wait for device ToF booted
        while (sensorState == 0)
        {
            status = VL53L1X_BootState(dev, &sensorState);
            SysTick_Wait10ms(10);
        }
        FlashAllLEDs();

        status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

        /* 2 Initialize the sensor with the default setting  */
        status = VL53L1X_SensorInit(dev);

        /* 3 Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
        // status = VL53L1X_SetDistanceMode(dev, 1); /* 1=short, 2=long */
        //  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
        //  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

        status = VL53L1X_StartRanging(dev); // 4 This function has to be called to enable the ranging
        FallingEdges == 1 ? UART_printf("0\n") : 1;
        float angle, x, y, z, prev_x, prev_y;
        int measured_steps = 64;
        z = FallingEdges * 100; 
        int half_step_sequence[8] = {
            0x01,
            0x03,
            0x02,
            0x06,
            0x04,
            0x0C,
            0x08,
            0x09};
        int step_index = 0; // Keep track of the current step in the sequence

        for (float i = 0; i < measured_steps; i++)
        { // Measure every 64 steps
            for (int j = 0; j < 4096 / measured_steps; j++)
            {
                GPIO_PORTH_DATA_R = half_step_sequence[step_index]; // Apply step pattern
                step_index = (step_index + 1) % 8;                  // Loop through half-step sequence
                SysTick_Wait(36000);
            }

            // 5 wait until the ToF sensor's data is ready
            while (dataReady == 0)
            {
                status = VL53L1X_CheckForDataReady(dev, &dataReady);
                VL53L1_WaitMs(dev, 5);
            }
            dataReady = 0;

            // 7 read the data values from ToF sensor
            status = VL53L1X_GetDistance(dev, &Distance); // 7 The Measured Distance value
            angle = 2*3.141592 * i / measured_steps;
            x = cos(angle) * Distance;
            y = sin(angle) * Distance;

            if (fabs(x) > 4000)
                x = prev_x;
            if (fabs(y) > 4000)
                y = prev_y;

            prev_x = x;
            prev_y = y;

            FlashLED4(1);

            status = VL53L1X_ClearInterrupt(dev); /* 8 clear interrupt has to be called to enable next interrupt*/

            // print the resulted readings to UART
            sprintf(printf_buffer, "%.3f %.3f %.3f  \r\n", x, y, z);
            UART_printf(printf_buffer);
            FlashLED3(1);
            SysTick_Wait10ms(1);
        }
        SysTick_Wait10ms(25);

        for (int j = 0; j < 4096; j++)
        {
            GPIO_PORTH_DATA_R = half_step_sequence[step_index]; // Apply step pattern
            step_index = (step_index - 1 + 8) % 8;              // Move in reverse order (counterclockwise)
            SysTick_Wait(20000);
        }

        SysTick_Wait10ms(1);
        VL53L1X_StopRanging(dev);
    }
}


// XSHUT     This pin is an active-low shutdown input;
//					the board pulls it up to VDD to enable the sensor by default.
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.

//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
int main(void)
{

    // initialize
    PLL_Init();
    SysTick_Init();
    onboardLEDs_Init();
    I2C_Init();
    UART_Init();
	
	
    PortH_Init();
    PortJ_Init(); 
    PortJ_Interrupt_Init();

    while (1)
    {
        OnLED1();
        WaitForInt();
    }
}