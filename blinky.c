// Hardik Patel
// Fadi Abdulhameed
// Casey Hood
// EEC 195B - Final Code - Winter 2017
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "inc/hw_gpio.h"
#include "driverlib/adc.h"
#include "inc/hw_nvic.h"
#include "driverlib/pwm.h"

/* Flags that contain the current value of the interrupt indicator as displayed on the UART. */
uint32_t g_ui32Flags;
/* The error routine that is called if the driver library encounters an error. */
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif
// Out constants for the timer freqency and PWM frequency.
#define FreqofTimer 100
#define PWM_freq 250
// Global variables.
volatile int count = 0; // Resets after filling the ping or pong buffer
volatile int pingbuffer[128], pongbuffer[128]; // Buffers to hold the camera value.
volatile int pingpong = 0; // Variable to indicate which buffer is full
volatile int32_t pui32ADC0Value[1]; // Gets the most recent camera value.
volatile int pingbufdone = 0, pongbufdone = 0; // THe full buffers flag is set to 1 and used in main.
volatile int speed = 1700; // The speed at which to run the car (input to PWM)
volatile int tempcount = 0; // Temporary variable used in testing
int temparray[128];
int temparray2[128]; // Array to hold the slope threshold calculations on the camera data.



/* Interrupt handler for Timer for the camera */
void Timer0IntHandler(void)
{
    ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); //  SI high
    ROM_GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7); // Used to measure the conversion time.
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // Clear interrupt
    ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4); // Clock High
    //Update global variables such as CLK counter (=1), Done flag(s), Buffer pointer, etc.
    count = 0; // Resetting the tracker for buffer values to 0
    ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0);  // SI low
    ADCProcessorTrigger(ADC0_BASE, 3); // Trigger ADC conversion
    ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0); // Clock low

}
/* Interrup handler for the ADC */
void ADCIntHandler(void){
    ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4); // Clock high
    ADCIntClear(ADC0_BASE, 3); // Clear ADC interrupt
    //Read A/D value
    ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value); // Getting the camera pixel value
    // Store A/D value in Ping Pong buffer appropriately
    if(pingpong == 0){
        pingbuffer[count] = pui32ADC0Value[0];
    }
    else if(pingpong == 1){
        pongbuffer[count] = pui32ADC0Value[0];
    }
    count++;
    tempcount++;
    if(count<129)
        ADCProcessorTrigger(ADC0_BASE, 3);// Trigger conversion till the buffer is full
    else{
        ROM_GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0); // To measure the full conversion time.
        // Setting the flags for the pingpong buffer
        if(pingpong==0){
            pingbufdone = 1;
            pingpong = 1;
        }
        else  {
            pongbufdone = 1;
            pingpong = 0;
        }
    }

    ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0); // Clock low

}
/* Configuring UART for UARTprintf() */
void ConfigureUART(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000);
}

/* To find the Maximum integer of an array */
int findMax(volatile int *arrayPtr)
{
    int Maximum = arrayPtr[0];
    int i =0;
    for(i =1; i <= 127; i++)
    {
        if(arrayPtr[i] > Maximum)
            Maximum = arrayPtr[i];
    }

    return Maximum;
}
/* To find the Minimum integer of an array */
int findMin(volatile int *arrayPtr)
{
    int Min = arrayPtr[0];
    int i =0;
    for(i =1; i <= 127; i++)
    {
        if(arrayPtr[i] < Min)
            Min = arrayPtr[i];
    }

    return Min;
}


/* Main */
int main(void)
 {
    ROM_FPULazyStackingEnable();
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                          SYSCTL_OSC_MAIN);
    ConfigureUART();

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_1);
    //Initialize timer
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    ROM_IntMasterEnable();
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet()/FreqofTimer);
    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0IntHandler);
    ROM_IntEnable(INT_TIMER0A);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Setting GPIO signals for the camera
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6); // SI
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4); // Clock
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7); // To measure conversion time
    // Initiallize ADC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE |
                             ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);
    ADCIntEnable(ADC0_BASE, 3);
    ADCIntRegister(ADC0_BASE, 3, ADCIntHandler);
    ADCIntEnable(ADC0_BASE, 3);
    IntMasterEnable();
    ROM_TimerEnable(TIMER0_BASE, TIMER_A);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2); // INB Setting
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3); // INA Setting
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);

    ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); //INA - Motor H bridge - high
    ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0); //INB - Motor H Bridge - low
    // Initialize PWM for servo and motor
    SysCtlPWMClockSet(SYSCTL_PWMDIV_16);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PC4_M0PWM6);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN |
                  PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN |
                                PWM_GEN_MODE_NO_SYNC);

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 12500);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 3125);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 4687); // Initialize servo to the middle
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, speed); // The spped of motor.
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);

    // Local constants for use in the while loop
    int max2 = 0, min2 = 0;
    int i = 0;
    int minflag = 0, maxflag = 0;
    int servodir = 0, servoconst = 54;
    int diff_in_index[4] = {0};
    int pterm = 0;
    int index[2];
    int errorcount = 0;
    int finaldterm = 0;
    float ptermconst = 0.7;
    float dterm = 0;
    float dtermconst = (0.4 /(6*FreqofTimer)) * 2500;
    float ptermpercent = 0.7, dtermpercent = 0.3;
    temparray2[0] = 0;
    temparray2[127] = 0;
    index[0] = 64;
    index[1] = 64;

    while(1)
    {

        while(!pingbufdone && !pongbufdone){ // Wait till one buffer is full
        }

        if(pingbufdone == 1 && pongbufdone ==0){ // When the ping buffer is full
            // Slope threshold method to finde the index.
            for(i = 1; i <127; i++){
                temparray2[i] = (pingbuffer[i+1] - pingbuffer[i-1]) >> 1;
            }
            max2 = findMax(temparray2);
            min2 = findMin(temparray2);
            for(i = 0; i <128; i++){
                if(temparray2[i] == max2)
                    maxflag = i;
                if(temparray2[i] == min2)
                    minflag = i;
            }
            index[errorcount%2] = (maxflag + minflag) >> 1; // Getting the correct index
            // Algorithm to fix the issue of car suddenly moving left while going right.
            if(abs(index[errorcount%2] - index[(errorcount+1)%2]) > 40)
                            index[errorcount%2] = index[(errorcount+1)%2];
            /*_______________________________Proportional___________________________*/
            diff_in_index[errorcount%4] = 64 - index[errorcount%2];
            pterm = ptermconst * diff_in_index[errorcount%4];
            /*______________________________________________________________________*/
            /*_______________________________Derivative_____________________________*/
            dterm = dtermconst * ((diff_in_index[errorcount%4] - diff_in_index[(errorcount + 1)%4]) + 3*(diff_in_index[(errorcount + 3)%4] - diff_in_index[(errorcount + 2)%4]));
            finaldterm = dterm;
            /*______________________________________________________________________*/
            // Calculate final servo direction
            servodir = 4687 + (((pterm * ptermpercent) +  (dtermpercent * finaldterm))* servoconst);
            // Statement to manage value fed in to the servo, not too high, not too low
            if (servodir > 6150){
                servodir = 6150;
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, speed*.1); // Reduce speed on sharp turns
            }
            else if (servodir < 3225){
                servodir = 3225;
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, speed*.1);
            }
            else
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, speed);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, servodir); // Set servo direction
            pingbufdone = 0; // Update flag
            errorcount++;

        }
        else if(pingbufdone == 0 && pongbufdone ==1){ // When the pong buffer is full
            // Slope threshold method to finde the index.
            for(i = 1; i <127; i++){
                temparray2[i] = (pongbuffer[i+1] - pongbuffer[i-1]) >> 1;
            }
            max2 = findMax(temparray2);
            min2 = findMin(temparray2);
            for(i = 0; i <128; i++){
                if(temparray2[i] == max2)
                    maxflag = i;
                if(temparray2[i] == min2)
                    minflag = i;
            }
            index[errorcount%2] = (maxflag + minflag) >> 1; // Getting the correct index
            // Algorithm to fix the issue of car suddenly moving left while going right.
            if(abs(index[errorcount%2] - index[(errorcount+1)%2]) > 40)
                           index[errorcount%2] = index[(errorcount+1)%2];

            /*_______________________________Proportional___________________________*/
            diff_in_index[errorcount%4] = 64 - index[errorcount%2];
            pterm = ptermconst * diff_in_index[errorcount%4];
            /*______________________________________________________________________*/
            /*_______________________________Derivative_____________________________*/
            dterm = dtermconst * ((diff_in_index[errorcount%4] - diff_in_index[(errorcount + 1)%4]) + 3*(diff_in_index[(errorcount + 3)%4] - diff_in_index[(errorcount + 2)%4]));
            finaldterm = dterm;
            /*______________________________________________________________________*/
            // Calculate final servo direction
            servodir = 4687 + (((pterm * ptermpercent) +  (dtermpercent * finaldterm))* servoconst);
            // Statement to manage value fed in to the servo, not too high, not too low
            if (servodir > 6250){
                servodir = 6150;
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, speed*.1); // Reduce speed on sharp turns
            }
            else if (servodir < 3125){
                servodir = 3225;
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, speed*.1);
            }
            else
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, speed);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, servodir); // Set servo directio
            pongbufdone = 0; // Update flag
            errorcount++;
        }
    }
}
