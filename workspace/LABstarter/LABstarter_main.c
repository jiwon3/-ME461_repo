//#############################################################################
// FILE:   LAB6_main.c
//
// TITLE:  Lab 6
//#############################################################################

// Included Files
//DESCRIPTION: In this lab we learn to use the DC motor speed control and steering a three wheel robot car
// We used previous LAB 5 code that read the MPU9250 chip and built on it. In this lab we focused more on controlling the robot with tera term
// where when we pressed respective buttons on the keyboard (which we coded into the F28379xD serial chip). An example would be pressing 4 to increase the speed or pressing
// s to stop it. We also were able to track where the robot was based on the x and y coordinates and the phi angle.
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void SPIB_isr(void);

void setEPWM2A(float controleffort);
void setEPWM2B(float controleffort);
void setEPWM3A(float controleffort);
void setupSpib(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

float ADCvolt1;
float ADCvolt2;

//Lab 5 stuff that was copied down.
float spivalue1 = 0;
float spivalue2 = 0;
float spivalue3 = 0;

int16_t UARTcount =0;
int16_t pwm_value =0;

int16_t dummy = 0;
int16_t accelXraw = 0;
int16_t accelYraw = 0;
int16_t accelZraw = 0;
int16_t gyroXraw = 0;
int16_t gyroYraw = 0;
int16_t gyroZraw = 0;

float accelXreading =0;
float accelYreading =0;
float accelZreading =0;
float gyroXreading =0;
float gyroYreading =0;
float gyroZreading =0;

// Lab 6 EX 1
//Here we made sure to initialize the functions that would be able to read and encode the left and right direction of the robot. The functions are just initialized here, and are actually
//written out later in the code. The function essentially should be returning the number of radians that the wheel has turned. This number will be used for future use.
void init_eQEPs(void);
float readEncLeft(void);
float readEncRight(void);

// We initialized the values for the left wheel and right wheel encoder positions, as well as the distances that they travel in their respective directions (which is based off of the gyroscope)
// to start at 0.
float LeftWheel = 0;
float RightWheel = 0;
float LeftDist = 0;
float RightDist = 0;

// LAB 6 EX 2
float uLeft = 5.0;
float uRight = 5.0;
// here we initialized the starting velocity of the left and right wheels.

// LAB 6 EX 3
float PosLeft_K = 0;
float PosLeft_K_1 = 0;
float PosRight_K = 0;
float PosRight_K_1 = 0;
float VRight_K = 0;
float VLeft_K = 0;
// So using the block diagram as the guide, we implmented the decoupled PI controller on both motors of the robot. We initially had set Ki to be 5, but then we changed it to 25 when we were
// validating the design. Each of the other variablees that would be used later in the equations was initialized to being 0 here. This helped since every time we would reset the robot in
// between trials, allowed us to go back to these set values.
// Every variable we used (except for gain), had a respective left and right. e is error here, and I is integrator.

float Kp = 3.0;
float Ki = 25.0;
float Vref = 0.0;
float Ik_left =0;
float Ik_1_left =0;
float ek_left =0;
float ek_1_left =0;
float Ik_right =0;
float Ik_1_right =0;
float ek_right =0;
float ek_1_right =0;

// Exercise 4
// We are continuing to intialize the variables here that will be used for future formulas. These variables here would be used for the calulation of the phi angle based on how much the
// the robot would be turning.
float P_turn = 0.5;
float e_turn = 0.25;
float turn = 0;
float K_turn = 3.0;

//Here, we are continuing initializing the variables that will be used for future formulas. One thing to note is that any of the ones that have a "_dot" essentially means that that is the
// derivative of.

//float w_r = 0.25;
float w_r = 0.56759; // radius of wheel
//float r_wh = 0.19460; this is the previous initial starting point for the radius
//float r_wh = 0.210; //this is what we iterated it to (too slow)
//float r_wh = 0.188;
float r_wh = 0.19; //width
float theta_r =0;
float theta_l =0;
float theta_avg =0;
float theta_r_dot =0;
float theta_l_dot =0;
float theta_r_k_1 = 0;
float theta_l_k_1 = 0;
float theta_avg_dot =0;
float phi_r =0;
float x_r =0; // x portion of radius
float y_r =0; // y portion of radius
float x_r_dot =0;
float y_r_dot =0;
float x_r_dot_k_1 =0;
float y_r_dot_k_1 =0;
float x_r_k_1 =0;
float y_r_k_1 =0;


void init_eQEPs(void) {
    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1; // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1; // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2; // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0; // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QEPCTL.bit.QPEN = 1; // Enable EQep

    // setup QEP2 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1; // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2B

    EQep2Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep2Regs.QPOSCNT = 0;
    EQep2Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
}
float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (raw*TWOPI/400/30*-1);
}
float readEncRight(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    // The function does not change between left or right, the only change is EQep1 vs EQep2
    return (raw*TWOPI/400/30);
}

__interrupt void SPIB_isr(void){

    dummy = SpibRegs.SPIRXBUF;
    accelXraw = SpibRegs.SPIRXBUF;
    accelYraw = SpibRegs.SPIRXBUF;
    accelZraw = SpibRegs.SPIRXBUF;
    dummy = SpibRegs.SPIRXBUF;
    gyroXraw = SpibRegs.SPIRXBUF;
    gyroYraw = SpibRegs.SPIRXBUF;
    gyroZraw = SpibRegs.SPIRXBUF;

    accelXreading = accelXraw*4.0*9.81/32767.0 -0.3; // acceleration in of m/s^2
    accelYreading = accelYraw*4.0*9.81/32767.0 +3.2;
    accelZreading = accelZraw*4.0*9.81/32767.0;
    gyroXreading = gyroXraw*250.0/32767.0;
    gyroYreading = gyroYraw*250.0/32767.0;
    gyroZreading = gyroZraw*250.0/32767.0;

//    if(accelYreading<-1.5){
//        Vref = 0;
//    }

    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt

    UARTcount ++;

    if (UARTcount == 200){   // the interrupt is called every 1 millisecond. We want the UART to print every 200ms. 200/1 = 200.
        UARTPrint = 1;
        UARTcount =0;
    }
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Set GPIO66 high to end Slave Select. Now Scope. Later to deselect DAN28027

    setEPWM3A(accelXreading);
}

uint16_t find_PWM_signal(void);
uint16_t counter = 0;

void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    // LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

    // LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

    //Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    PieVectTable.SPIB_RX_INT = &SPIB_isr;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 1000);
    // For EX3, we changed to 20000 (20 ms) from its original of 10000 (which is 10 ms)
    // For EX4, we changed it again to 1000 (which is 1 ms). This is because the period is given in microseconds, therefore one needs to convert it from milliseconds to microseconds.
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 4000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    init_eQEPs();
    GPIO_SetupPinMux(2,GPIO_MUX_CPU1,1); // pinmux to epwm2a
    GPIO_SetupPinMux(3,GPIO_MUX_CPU1,1); // pinmux to epwm2b
    GPIO_SetupPinMux(4,GPIO_MUX_CPU1,1); // pinmux to epwm3a
    setupSpib();

    EPwm2Regs.TBCTL.bit.CTRMODE = 0; // Count up
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 2; // FREE SOFT to free run
    EPwm2Regs.TBCTL.bit.CLKDIV = 0; // Clock divide by 1
    EPwm2Regs.TBCTL.bit.PHSEN = 0; // Phase loading disabled

    EPwm2Regs.TBCTR = 0; // start counter at 0

    EPwm2Regs.TBPRD = 2500; // 50us = period/50mil -> period = 2500

    EPwm2Regs.CMPA.bit.CMPA = 0; // duty cycle 0%
    EPwm2Regs.AQCTLA.bit.CAU = 1; // signal pin clear when tbctr = cmpa
    EPwm2Regs.AQCTLA.bit.ZRO = 2; // when TBCTR = 0, set force EPWMxA high
    EPwm2Regs.CMPB.bit.CMPB = 0; // duty cycle 0%
    EPwm2Regs.AQCTLB.bit.CBU = 1; // signal pin clear when tbctr = cmpa
    EPwm2Regs.AQCTLB.bit.ZRO = 2; // when TBCTR = 0, set force EPWMxA high

    EPwm2Regs.TBPHS.bit.TBPHS = 0; // set the phase to zero

    // copy and paste epwm2
    EPwm3Regs.TBCTL.bit.CTRMODE = 0; // Count up
    EPwm3Regs.TBCTL.bit.FREE_SOFT = 2; // FREE SOFT to free run
    EPwm3Regs.TBCTL.bit.CLKDIV = 0; // Clock divide by 1
    EPwm3Regs.TBCTL.bit.PHSEN = 0; // Phase loading disabled

    EPwm3Regs.TBCTR = 0; // start counter at 0

    EPwm3Regs.TBPRD = 2500; // 50us = period/50mil -> period = 2500

    EPwm3Regs.CMPA.bit.CMPA = 0; // duty cycle 0%
    EPwm3Regs.AQCTLA.bit.CAU = 1; // signal pin clear when tbctr = cmpa
    EPwm3Regs.AQCTLA.bit.ZRO = 2; // when TBCTR = 0, set force EPWMxA high
    EPwm3Regs.CMPB.bit.CMPB = 0; // duty cycle 0%
    EPwm3Regs.AQCTLB.bit.CBU = 1; // signal pin clear when tbctr = cmpa
    EPwm3Regs.AQCTLB.bit.ZRO = 2; // when TBCTR = 0, set force EPWMxA high

    EPwm3Regs.TBPHS.bit.TBPHS = 0; // set the phase to zero

    EALLOW; // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1; // For EPWM2A
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1; // For EPWM2B
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1; // For EPWM3A
    EDIS;

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;
    IER |= M_INT6; // SPIB for ex2 setup


    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

    //Exercise 2
    // Enable SPIB in the PIE: Group 6 interrupt 3
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;




    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {

            serial_printf(&SerialA,"accelX:%.3f accelY: %.3f accelZ:%.3f gyroX:%.3f gyroY: %.3f gyroZ: %.3f \r\n", accelXreading, accelYreading, accelZreading, gyroXreading, gyroYreading, gyroZreading);
            //serial_printf(&SerialA,"LeftWheel:%.3f RightWheel: %.3f LeftDist: %.3f RightDist: %.3f LeftVel: %.3f RightVel: %.3f x_r: %.3f y_r: %.3f x_r_dot: %.3f y_r_dot: %.3f phi_r: %.3f \r\n", LeftWheel,RightWheel,LeftDist,RightDist,VLeft_K,VRight_K,x_r,y_r,x_r_dot,y_r_dot,phi_r);
            UARTPrint = 0;
        }
    }
}
// Above we are trying to print out the location of the left and right wheel, as well as the distance each of the wheels has traveled. In addition, we want to output the left and right wheel
// velocities, as well as phi angle it has turned. This is based off of its x and y coordinates (which is based off of the radius).
// One thing to note is when we are trying to start at a certain location, traverse a particular distance (say a square box), and then come back to the intitial location -
// we would expect the coordinates of x and y to go back to 0, and the phi angle to go to 2pi (since it turned a full circle).


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts



    // Insert SWI ISR Code here.......


    numSWIcalls++;

    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;
    // Exercise 3
    // Call two functions to generate a number between 0-3000 through incrementation.
    //    int16_t PWM1 = find_PWM_signal();
    //    int16_t PWM2 = find_PWM_signal();


    //Exercise2
    // Configure SPIB to send and receive messages through SPI.
    //Clear GPIO9 Low to act as a Slave Select. Right now, just to scope. Later to select DAN28027 chip
    // GpioDataRegs.GPACLEAR.bit.GPIO9 = 1; //GPA because we're using pin number 9. GPACLEAR because we want to set it low.
    // SpibRegs.SPIFFRX.bit.RXFFIL = 3; // Issue the SPIB_RX_INT when three values are in the RX FIFO
    // SpibRegs.SPITXBUF = 0x4A3B; // 0x4A3B and 0xB517 have no special meaning. Wanted to send
    // SpibRegs.SPITXBUF = 0xB517; // something so you can see the pattern on the Oscilloscope

    //Exercise3
    // SpibRegs.SPITXBUF = 0x00DA; // always sent when sending signals on the SPI bus
    // SpibRegs.SPITXBUF = PWM1; // set the output of send_PWM1_signal() to be the SPI TX buffer register
    // SpibRegs.SPITXBUF = PWM2; // set the output of send_PWM2_signal() to be the SPI TX buffer register

    //Exercise4
    //Open the MPU-9250 Register Map and find that the register (ACCEL_XOUT_L) is located at the address 0x3C and the registers ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L are next in line at 0x3D, 0x3E, 0x3F and 0x40.

    //ACCEL XYZ
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; //GPC because we're using number 66
    SpibRegs.SPIFFRX.bit.RXFFIL = 8;
    // X HIGH AND LOW
    SpibRegs.SPITXBUF = ((0x8000)|(0x3A00)); // Use or operator with 0x8000 to command read
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;

    //GpioDataRegs.GPCSET.bit.GPIO66 = 1;


    //    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; //GPC because we're using number 66
    //    SpibRegs.SPIFFRX.bit.RXFFIL = 3;
    //    SpibRegs.SPITXBUF = ((0x8000) | (0x3C00));
    //    SpibRegs.SPITXBUF = 0;
    //    SpibRegs.SPITXBUF = 0;
    //    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    //    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    //    temp = SpibRegs.SPIRXBUF;
    //    DELAY_US(10);


    //    if ((numTimer0calls%50) == 0) {
    //        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
    //    }

    if ((numTimer0calls%250) == 0) {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }



    // Blink LaunchPad Red LED
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;




}

// since timer0 was already being used from a previous lab, we must use timer1 here.
// Per exercise one, we need it to read the left and right location of the wheel every 4 pm.
// We are dividing by 5 to calculate the distance (this was determined via trial and error).
// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    // Exercise 1 - every 4ms read left and right

    LeftWheel = readEncLeft();
    RightWheel = readEncRight();
    LeftDist = LeftWheel/5.0;
    RightDist = RightWheel/5.0;

    // Here, we are just typing in the formulas that were given in the lab manual using the different positions to find the velocity. The reason we are dividing by 0.004 is because we should be
    // running it every 4 ms, and if we do a trapezoidal approach to integration, that would be time between the current term and the term before or after it (aka the "height" in the trapezoid)

    PosLeft_K = LeftDist;
    VLeft_K = (PosLeft_K-PosLeft_K_1)/0.004;

    // Again, note that whatever we do on the left wheel, we do on the right wheel as well.
    PosRight_K = RightDist;
    VRight_K = (PosRight_K-PosRight_K_1)/0.004;

    theta_r = RightWheel;
    theta_l = LeftWheel;

    // Below, we are continuing to copy the formulas that were given to us from the block diagram to implement the decoupled PI controller on both motors of the robot.
    //ek_left = Vref - VLeft_K; // sed in exercise 3 in lab 6.
    // The following formulas are used for exercise 4 in lab 6.
    // We must first define was the error in turn is, which is dependent on the current turn and the velocities of each of the wheels.
    // After that, we can determine the positional errors for the left and right wheels, which is dependent on the reference velocity, as well as the velocity on the respective wheel and
    // the positional gain and error of the wheel
    e_turn = turn+(VLeft_K-VRight_K);
    ek_left = Vref - VLeft_K-K_turn*e_turn;
    Ik_left = Ik_1_left + 0.004*(ek_left + ek_1_left)/2;
    uLeft = Kp*ek_left + Ki*Ik_left;

    // Here, we are essentially just iterating the variables so that we can get the previous term when a new velocity term has been created.
    if(uLeft > 10){
        uLeft =10;
        Ik_left = Ik_1_left;
    }
    if(uLeft < -10){
        uLeft =-10;
        Ik_left = Ik_1_left;
    }

    //ek_right = Vref - VRight_K; // Exercise 3

    // the same information that was mentioned above for the left wheel velocity is now essentially being copied here, but for the right wheel. The logic overall is essentially the same.
    ek_right = Vref - VRight_K+K_turn*e_turn;
    Ik_right = Ik_1_right + 0.004*(ek_right + ek_1_right)/2;
    uRight = Kp*ek_right + Ki*Ik_right;
    if(uRight > 10){
        uRight =10;
        Ik_right = Ik_1_right;
    }
    if(uRight < -10){
        uRight = -10;
        Ik_right = Ik_1_right;
    }
    //So this was determined by zeroing one of them out (we zeroed out uLeft) to determine which EPWM goes with which of them. After that, we had to make one of them negative, again we had to do
    // do uLeft so that they both go in the same direction are both going forward. If we were to instead negate uRight, they would both go in the same direction, but the robot would be going
    // backwards, which is not what we need here.
    setEPWM2B(-uLeft);
    setEPWM2A(uRight);


    PosLeft_K_1 = PosLeft_K;
    PosRight_K_1 = PosRight_K;
    Ik_1_left = Ik_left;
    Ik_1_right = Ik_right;
    ek_1_left = ek_left;
    ek_1_right = ek_right;

    // Exercise 5
    // Here, we are just typing in the formulas that were given in the lab manual using the different positions to find the theta angle. The reason we are dividing by 0.004 is because we should be
    // running it every 4 ms, and if we do a trapezoidal approach to integration, that would be time between the current term and the term before or after it (aka the "height" in the trapezoid)


    theta_r_dot = (theta_r-theta_r_k_1)/0.004; //Finding the differential using k-1 values defined at the end of the code
    theta_l_dot = (theta_l-theta_l_k_1)/0.004;

    //the phi angle is dependent on r_wh, which is the width, and w_r, which is the radius of the wheel.
    phi_r = r_wh/w_r*(theta_r - theta_l);
    // we are applying just the normal average formula which is you add the two components you are averaging, and then divide it by 2.
    theta_avg =  0.5*(theta_r+theta_l);
    // Here, you are using the same logic as above, but for its differentiated (first derivative) components.
    theta_avg_dot = 0.5*(theta_r_dot + theta_r_dot);

    // Here, we are essentially doing x = r*costheta, and y = r*sintheta, which essentially gives us the x and y components based of the radial velocity and the phi angle, which were
    // both previously calculated and explained
    x_r_dot = r_wh*theta_avg_dot*cos(phi_r);
    y_r_dot = r_wh*theta_avg_dot*sin(phi_r);

    // Here, we are just typing in the formulas that were given in the lab manual using the different positions to find the x and y radial components. The reason we are dividing by 0.004 is because we should be
    // running it every 4 ms, and if we do a trapezoidal approach to integration, that would be time between the current term and the term before or after it (aka the "height" in the trapezoid)

    x_r = (x_r_dot + x_r_dot_k_1)*0.004/2 + x_r_k_1;
    y_r = (y_r_dot + y_r_dot_k_1)*0.004/2 + y_r_k_1;

    // this allows us to iterate where the previous term becomes the next term.
    theta_r_k_1 = theta_r;
    theta_l_k_1 = theta_l;
    x_r_dot_k_1 = x_r_dot;
    y_r_dot_k_1 = y_r_dot;
    x_r_k_1 = x_r;
    y_r_k_1 = y_r;

    CpuTimer1.InterruptCount++;


}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;

    if ((CpuTimer2.InterruptCount % 50) == 0) {
        //       UARTPrint = 1;
    }
};

//Exercise 4
void setupSpib(void) //Call this function in main() somewhere after the DINT; line of code.
{
    int16_t temp = 0;

    //Step 1.
    // cut and paste here all the SpibRegs initializations you found for part 3. Make sure the TXdelay in

    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0); // Set as GPIO9 and used as DAN28027 SS
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO9 an Output Pin
    GpioDataRegs.GPASET.bit.GPIO9 = 1; //Initially Set GPIO9/SS High so DAN28027 is not selected
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB//Found from pin mux table
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB//Found from pin mux table
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB//Found from pin mux table
    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI

    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset
    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 15; // Set to transmit and receive 16 bits each write to SPITXBUF
    SpibRegs.SPICTL.bit.TALK = 1; // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is // 50MHZ. And this setting divides that base clock to create SCLK�s period
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
    SpibRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
    SpibRegs.SPIFFCT.bit.TXDLY = 0; //Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip
    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset
    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don�t think this is needed. Need to Test
    SpibRegs.SPIFFRX.bit.RXFFIL =16; //Interrupt Level to 16 words or more received into FIFO causes for Exercise 2
    //between each transfer to 0. Also don�t forget to cut and paste the GPIO settings for GPIO9, 63, 64, 65,
    //66 which are also a part of the SPIB setup.
    SpibRegs.SPICCR.bit.SPICHAR = 0xF;
    SpibRegs.SPIFFCT.bit.TXDLY = 0x00;
    //-----------------------------------------------------------------------------------------------------------------

    //Step 2.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    SpibRegs.SPITXBUF = (0x1300 | 0x0000); // Address 00x13 and value 0x00
    SpibRegs.SPITXBUF = (0x0000 | 0x0000); // Writing values for 0x14 and 0x15
    SpibRegs.SPITXBUF = (0x0000 | 0x0000); // Writing values for 0x16 and 0x17
    SpibRegs.SPITXBUF = (0x0000 | 0x0013); // Writing values for 0x18 and 0x19
    SpibRegs.SPITXBUF = (0x0200 | 0x0000); // Writing values for 0x1A and 0x1B
    SpibRegs.SPITXBUF = (0x0800 | 0x0006); //Writing values for 0x1C and 0x1D
    SpibRegs.SPITXBUF = (0x0000 | 0x0000);// Writing values for 0x1E and 0x1F
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave select high


    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are
    // sending 16 bit transfers, so two registers at a time after the first 16 bit transfer.

    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST !=7);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF; // read 7 garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    //Step 3.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    SpibRegs.SPITXBUF = (0x2300 | 0x0000); // Address 00x23 and value 0x00
    SpibRegs.SPITXBUF = (0x4000 | 0x008C); // Writing values for 0x24 and 0x25
    SpibRegs.SPITXBUF = (0x0200 | 0x0088); // Writing values for 0x26 and 0x27
    SpibRegs.SPITXBUF = (0x0C00 | 0x000A); // Writing values for 0x28 and 0x29

    while(SpibRegs.SPIFFRX.bit.RXFFST !=4);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High

    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;// read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    //Step 4.
    // perform a single 16 bit transfer to initialize MPU-9250 register 0x2A

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x2A00 | 0x0081);// Write to address 0x2A the value 0x81
    // wait for one byte to be received
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    // The Remainder of this code is given to you and you do not need to make any changes.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3800 | 0x0001); // 0x3800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3A00 | 0x0001); // 0x3A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6400 | 0x0001); // 0x6400
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6A00 | 0x0020); // 0x6A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7500 | 0x0071); // 0x7500

    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7700 | 0x00E6); // 0x7700 // first two hexadecimal chars of offset XA, notes on how we got this number below
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | 0x00EC); // 0x7800 // our first number was -2679 decimal, but we discovered we needed an extra offset of 531
    //we left shifted this pentadecimal to hexadecimal and got E6EC
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | 0x00E9); // 0x7A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | 0x0006); // 0x7B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x001A); // 0x7D00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | 0x00E8); // 0x7E00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);
    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

// process to finding the offset
// we first used the teraterm output and found what the values were around, and added the number to the initial offset given with the lab instructioons.
// We divIded the total offset by 0.00098 to get a value
// after that, we subtracted it from the initial offset value, to get the new offset value
// finally, we converted that value to hex and left shifted it by 1 bit
// finally we inputted that into the respective spibRegs.SPITXBUF

//functions for EPWM2A and EPWM2B from lab 3
void setEPWM2A(float controleffort){
    float x =0;

    if (controleffort>10){
        x=10;
    }
    else if(controleffort<-10){
        x=-10;
    }
    else {
        x=controleffort;
    }

    float y = 0;
    // linearly scale -10 0 10 to 0 1250 2500
    // multiply by 1250/10 and add 1250
    y = 125.0*x+1250.0;

    EPwm2Regs.CMPA.bit.CMPA = y;
}

void setEPWM2B(float controleffort){
    float x = 0.0;

    if (controleffort>10){
        x=10;
    }
    else if(controleffort<-10){
        x=-10;
    }
    else {
        x=controleffort;
    }


    float y = 0.0;
    // linearly scale -10 0 10 to 0 1250 2500
    // multiply by 1250/10 and add 1250
    y = 125.0*x+1250.0;

    EPwm2Regs.CMPB.bit.CMPB = y;
}

void setEPWM3A(float controleffort){
    float x = 0.0;
//
//        if (controleffort>4){
//            x=4;
//        }
//        else if(controleffort<-4){
//            x=-4;
//        }
//        else {
//            x=controleffort;
//        }

        float y = 0.0;
        // linearly scale -4 0 4 to 0 1 2
        // multiply by 1250/10 and add 1250
        //y = x/4+1;
        y= controleffort;

        EPwm3Regs.CMPA.bit.CMPA = controleffort;

}

// In this lab we gained more exposure to the robot car. It was honestly in some ways more complicated than we had anticipated, but it was still very interesting. Taking other classes where
// we have to decipher block diagrams and apply PID controllers, it was really intriguing to be able to apply prior knowledge to this lab.
// I enjoyed playing with the code in the F28379xD serial chip where as one clicks a certain key, the robot acts in a particular way. For example we did it so that pressing the button s
// would increase the robot's speed, while pressing buttons c and v would increase or decrease the robot's radius of the wheel for the calculations.
// Doing this lab would very helpful since we will definitely use the knowledge we learned, and apply it towards our final project since we plan on creating an autonomous robot car.
