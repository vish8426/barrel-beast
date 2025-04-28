#include <stdio.h>
#include <stdlib.h>
#include "ConfigRegsPIC18F452.h"
#include <p18f452.h>
#include <string.h>
#include "delays.h"
#include <usart.h>
#include <math.h>
#define TRUE 1 
#define FALSE 0 


//////////////////////////////////////////////////////////////////////////////
// XBee variables and functions
/**
    configValues is a struct used to store configuration values entered by
    user on the controller menue interface.
*/
typedef union{   
    struct{       
        unsigned char pid_p;
        unsigned char pid_i;
        unsigned char pid_d;
        unsigned char yaw;
        unsigned char display;
        unsigned char max_speed;
        unsigned char IR_samples;
        unsigned char IR_rate;
        unsigned char mode;
    };   
    unsigned char array[8];
}ConfigValues;
ConfigValues config;

/**
    Vector is a struct used to store joystick values once transmitted over
    USART and xBee communication.
*/
typedef struct{
    signed char x;
    signed char y;
    char z;
}Vector;
Vector velocity;

// XBee Global Variables
int i = 0;                  // Index counter for receiving strings
char readValues[4];         // Receive Character Buffer
int startReading = FALSE;   // Start Reading flag

/**
    configureUART sets the appropriate configuration bits inside the TXSTA
    and RCSTAbits registers in order for USART to function correctly as per
    project requirements. PORTC6 is set as an output to enable Transmission
    and PORTC7 is set as an input to enable receiving. This function must be
    called before receiving or transmitting strings via USART using xBeeSend
    and xBeeOnReceive.
    
        @see xBeeOnReceive & xBeeSend
*/
void configureUART(void){
    
    TXSTAbits.BRGH = 1; // High speed mode (y=16)
    SPBRG = 64;         // X = (10^7/9600*16) - 1 = 64.1
    TXSTAbits.SYNC = 0; // Asynchronous mode
    RCSTAbits.SPEN = 1; // Enable serial port
   
    TXSTAbits.TX9  = 0;  // Enable 8 bit transmission
    TXSTAbits.TXEN = 1; // Enable transmit
    RCSTAbits.RX9  = 0;  // 9 bit receive
    RCSTAbits.CREN = 1; // receive enabled
    
    TRISCbits.RC6  = 0;  // Enable Pin 6 as Tx
    TRISCbits.RC7  = 1;  // Enable pin 7 as Rx
    
}

/**
    configureUARTInterrupts sets the appropriate configuration bits in order
    to enable interrupt on receive for the system. This function must be
    called before xBeeOnReceive.
        @see xBeeOnReceive
*/
void configureUARTInterrupts(void){
   
    INTCONbits.GIEH = 0; // Disable Interrupts
    INTCONbits.GIEL = 0; // Disable Peripheral interrupts 
    RCONbits.IPEN   = 1;   // Enable Priority
    PIE1bits.RCIE   = 1;   // Enable USART Receive Interrupt
    IPR1bits.RCIP   = 1;   // Enable High Priority on receive interrupt *
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1; 
    INTCONbits.PEIE = 1; // Re-enable interrupts
   
}

/**
    xBeeSend is used to send strings from the robot to controller wirelessly
    over USART and xBee transmission.
    
        @param string, the string to be sent
*/
void xBeeSend(char * string){
    char c;   
    // Loop through all characters in string
    while(c = *(string ++)){
        while(PIR1bits.TXIF == 0);     // Wait till transmit buffer is clear
        TXREG = c;                     // Transmit character over USART
    }

}

/**
    saveJoyStickValues saves the x,y and z joystick values into the velocity
    struct if the appropriate start bit has been read.
*/
void saveJoyStickValues(void){
    velocity.x = readValues[1]; // Save buffer position 1 as x
    velocity.y = readValues[2]; // Save buffer position 2 as y
    velocity.z = readValues[3]; // Save buffer position 3 as z
}

/**
    saveConfigValues saves configuration values into the correct index
    position of the configuration array defined in the config union.
*/
void saveConfigValues(void){
    // Save value in buffer position 2 into array position dictated by buffer
    // position 1.
    config.array[readValues[1]] = readValues[2];
}

/**
    xBeeOnReceive is used to declare the USART receive interrupt and declare
    the ISR in the correct place in memory. This function must be called for
    the highPriorityISR to function correcty.
    
        @see highPriorityISR
*/
void highPriorityISR(void);    // Function declaration for highPriorityISR
#pragma code highPriAddr = 0x08
void xBeeOnReceive(void){
    _asm goto highPriorityISR _endasm
}
#pragma code

/**
    highPriorityISR is the Interrupt Service Routine involved with recieving
    configuration values and storing them in the correct variables. This
    function uses saveConfigValues & saveConfigValues to store received
    strings correctly.
    
        @see saveJoyStickValues & saveConfigValues
*/
#pragma interrupt highPriorityISR
void highPriorityISR(void){
   
    // Read character from receive register
    char currentChar = RCREG;   
    
    // Check if one of the start bits as been read
    if((currentChar == 'V'||currentChar == 'C') && startReading == FALSE){
        
        // Begin reading if start bit found
        i = 0;
        startReading = TRUE;            
        readValues[i] = currentChar;
        i++;
        return;
    }

    // If start reading flag is off dont read character.
    // This stops errors in transmission effecting output variables
    if(startReading == FALSE)return;
    
    // Save current character into read buffer
    readValues[i] = currentChar;
    i++;

    // If 3 characters have been read after stop bit
    if(i == 3){

        // If start bit was V joystick values were sent and save buffer into
        // velocity struct
        if(readValues[0] == 'V'){
            saveJoyStickValues(); 

        }      
        
        // If start bit was C config values were sent and save buffer into
        // config array
        if(readValues[0] == 'C'){
            saveConfigValues();
        }
        
        // Reset start reading flag and index
        startReading = FALSE;
        i = 0;     
    }
}
#pragma code
// End XBee variables and functions
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Motor variables and functions
/**
    Struct to store left and right power values for motor
*/
typedef struct
{
    char left;
    char right;
} Motor;

Motor power;                // Global struct for power form joysticks
Motor IR_power;             // Global struct for power from ir_sensors

// Global variables
float speed; 
float speedscale;
float max_speed;
unsigned int max_yaw = 100;
float yawscale;

/**
    vectorToMotor converts a vector received from the joysticks to a power
    value that can be used in further calculations or sent to the motors as
    output with the setMotors function.

        @see setMotors

        @param v, the vector from the joysticks

        @return The left and right power values from 0-127 to be sent to the
        motors.
*/
Motor vectorToMotor(Vector v){
    Vector output;
    Motor motor_out;
    int xadjust; 
    int yadjust;
    
        
    // scale v.x to a percentage according to max yaw/ 100 
    xadjust = v.x;
    xadjust = xadjust * config.yaw / 100; 
    v.x = -xadjust; 
    
    // scale v.y to a percentage according to max speed/100
    yadjust = v.y;
    yadjust = yadjust * config.max_speed / 100;
    v.y = yadjust;
    
   
    // if there is no x componet there is no need to scale 
     if (v.x == 0)
    {
        output.x = v.x;
        output.y = v.y;
    }
    else
    {
         // two different equations to allow for greater control in fringe
         // cases obtain x offset from centres effect on each wheel and
         //calculate y for average speed of wheels 
        if((fabs(v.x)/fabs(v.y))>1){

            output.x = (fabs(v.x) * v.x) / (fabs(v.x) + fabs(v.y));
            output.y = (fabs(v.x) * v.y) / (fabs(v.x) + fabs(v.y));

        }
        else{
            output.x = (fabs(v.y) * v.x) / (fabs(v.x) + fabs(v.y));
            output.y = (fabs(v.y) * v.y) / (fabs(v.x) + fabs(v.y));
        }
    }
    // offset speeds values by the calculated x offset 
    motor_out.left = output.y + output.x;
    motor_out.right = output.y - output.x;

    return motor_out;
}

/**
    setMotors takes a Motor struct with values for the left and right power
    and sends them to the appropriate pins to output power to the motors.
*/
void setMotors(Motor power){
    char left;
    char right;
    // set pins based on direction required and make values positive if 
    // required for each wheel 
    if (power.left < 0){

        // make positive
        left = power.left * -1;

       //set INA to 0 and INB to 1 to reverse
       PORTDbits.RD3 = 0;
       PORTDbits.RD4 = 1;


    }else{

        
        left = power.left;
        //set INA to 0 and INB to 1 to reverse
       PORTDbits.RD3 = 1;
       PORTDbits.RD4 = 0;
       
   
    }if (power.right < 0){

        // make positive
        right = power.right * -1;

       //set INA to 0 and INB to 1 to reverse
       PORTDbits.RD1 = 0;
       PORTDbits.RD2 = 1;


    }else{

       
        right = power.right;
        //set INA to 0 and INB to 1 to reverse    
        PORTDbits.RD1 = 1;
        PORTDbits.RD2 = 0;
    }

    //left=left/4;
    //right=right/4;

    //add least significant bits to the low duty registers
    CCP1CONbits.DC1B = left;
    CCP2CONbits.DC2B = right;

    //left = left >> 2;
    //right = right >> 2;

    CCPR1L = left; //left motor output to RC1 for 8 MSB

    CCPR2L = right; //Right motor output to RB3 for 8 MSB
}

/**
    motorsSetup sets up the necessary pins as input and output, and sets
    up the CCP module to output a PWM signal to the motors.
*/
void motorsSetup(){
    TRISD = 0x00; //set D all outputs for LED
    
    TRISCbits.RC1 = 0;// set RC1 to output 
    TRISCbits.RC2 = 0;// set RC2 to output 
    TRISCbits.CCP2 = 1; 
    TRISBbits.CCP2 = 1; 
    TRISBbits.RB3 = 0;// set RB3 to output for PWM


    //timer 2 on and prescale 4
    T2CONbits.TMR2ON = 1;
    T2CONbits.T2CKPS0 = 1;
    T2CONbits.T2CKPS1 =0;


    // PWM setup
    CCP1CONbits.CCP1M3 = 1;
    CCP1CONbits.CCP1M2 = 1;

    CCP2CONbits.CCP2M3 = 1;
    CCP2CONbits.CCP2M2 = 1;
    CCP2CONbits.CCP2M1 = 1;
    CCP2CONbits.CCP2M0 = 1;


    // make PR2 to 127 for 4.8khz pwm duty to fit announcement
    PR2 = 0x7F;
}
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// IR variables and functions
// IR Macros
#define IR_BUFFER_MAX 40            // Maximum buffer size for IR samples
#define WHEELDIST 200               // Wheelbase in mm
#define TURNING_R 220               // Turning radius
#define TURNING_S 50                // Turning speed
#define IR_MIN_SAMPLE 655           // Minimum number of IR samples

// IR global variables
unsigned char sample_multiplier = 1;    // Multiplier for configuring sample
                                        // rate
unsigned short int sample_rate = 0;         // IR sample rate
unsigned char sample_rate_H = 0;            // For high byte TMR0H
unsigned char sample_rate_L = 0;            // For low byte TMR0L

unsigned char irBufferLeft[IR_BUFFER_MAX];  // Buffer for left IR values
unsigned char irBufferRight[IR_BUFFER_MAX]; // Buffer for right IR values

unsigned char irBufferLeftHead = 0;     // Buffer for reading left IR value
unsigned char irBufferRightHead = 0;    // Buffer for reading right IR value

// User assist globals
char UAFlag = 0;                // User assist flag for which side IR sensors
                                // have detected an object
char mode = 1;              // Flag for current mode of operation

/**
    PIDVariables is struct used for storing the PID variables for use in 
    IR PID calculation.
*/
typedef struct {
    char desired_value;
    char IR;
    char last_error;
    float current_desired_variable;
    char error;
    float integral;
    float derivative;
}PIDVariables;
PIDVariables IR_PID;

/**
    IRResult is struct used to store left and right IR values.
*/
typedef struct{
    unsigned char left;
    unsigned char right;
} IRResult;

IRResult readings;

/**
    initADC sets PORTA as input for analog input.
*/
void initADC(void){
    TRISA  = 0xff;            // PORT A Set as Input Port
}

/**
    analogRead reads a voltage form a specified channel and returns
    the voltage in 8 bit resolution.

        @param  channel, the pin on port A to be read (i.e. 0 - 7).

        @return The voltage on the pin in 8 bit resolution.
*/
unsigned char analogRead(char channel){
    ADCON0bits.ADCS = 1;      // Fosc/8 clock conversion
    ADCON1bits.ADCS2 = 0;      // Fosc/8 clock conversion

    ADCON1bits.ADFM = 0;      // Left Justified
    ADCON1bits.PCFG = 0b1001; // Use all 6 analog pins and Vdd and Vcc as ref

    ADCON0bits.CHS = channel; // Set channel

    ADCON0bits.ADON = 1;      // Power up ADC

    Delay10TCYx(15);         // wait acquisition time

    ADCON0bits.GO = 1;        // Start AD Conversion

    while(ADCON0bits.GO == 1);// Wait for ADC Conversion to complete

    return ADRESH;            // Output ADC Result
}

/**
    TMR0Setup sets up Timer1 as a 16-bit timer with a prescale of 1:2 and an
    initial delay dependent on the input IR sample rate from the commander.
    Timer0 is started at the end of TMR0Setup to begin sampling.
*/
void TMR0Setup(void) {
    T0CONbits.TMR0ON = 0;       // Start timer off
    T0CONbits.T08BIT = 0;       // 16-bit
    T0CONbits.T0CS = 0;         // internal clock
    T0CONbits.PSA = 0;          // use prescaler
    T0CONbits.T0PS2 = 0;        // 1:2 prescaler
    T0CONbits.T0PS1 = 0;
    T0CONbits.T0PS0 = 0;
    
    sample_multiplier = config.IR_rate;
    sample_rate = 65535 - sample_multiplier * IR_MIN_SAMPLE;
    sample_rate_H = sample_rate >> 8;
    sample_rate_L = sample_rate & 0xFF;
    
    TMR0H = sample_rate_H;
    TMR0L = sample_rate_L;
    
    T0CONbits.TMR0ON = 1;        // Turn timer on
}

/**
    irReadValues reads values from the right and left IR sensors on PORTA pins
    2 and 3 using the function analogRead.

        @see analogRead
*/
void irReadValues(void){
    unsigned char right = analogRead(2); 
    unsigned char left = analogRead(3);

    irBufferLeftHead =  ( irBufferLeftHead +  1 ) % config.IR_samples;
    irBufferRightHead = ( irBufferRightHead + 1 ) % config.IR_samples;
    
    irBufferLeft[irBufferLeftHead] = left;
    irBufferRight[irBufferRightHead] = right;
}

/**
    irGetMean takes the stored IR values over the sample period and calculates
    the mean and returns the value as an IRResult struct.

        @return IRResult struct of left and right IR values.
*/
IRResult irGetMean(void){
    IRResult result;
    unsigned int sumLeft = 0;    
    unsigned int sumRight = 0;

    unsigned char i;
    for (i = 0; i < config.IR_samples; i++) {
        sumLeft += irBufferLeft[i];
        sumRight += irBufferRight[i];
    }
    result.left = 180 - sumLeft/(config.IR_samples);    
    result.right = 180 - sumRight/(config.IR_samples);
    return result;
}

/**
    irPID uses proportional control to adjust the power that is output to the
    motors by the setMotors function.

        @see setMotors
*/
void irPID(void){
    float kp_ir = 0.1;
    float ki_ir = 0;
    int kd_ir = 0;

   // Find error
      IR_PID.error = IR_PID.desired_value - IR_PID.IR;
   // Calculate integral controller
      IR_PID.integral = IR_PID.integral + IR_PID.error;

   // Calculate derivative controller
      IR_PID.derivative = IR_PID.error - IR_PID.last_error;

   // Calculate value closer to the desired variable
      IR_PID.current_desired_variable = (kp_ir*IR_PID.error) +
      (ki_ir*IR_PID.integral) + (kd_ir*IR_PID.derivative);

   // Store previous error
      IR_PID.last_error = IR_PID.error;
}

/**
    irToMotor converts the PID output from the irPID function to a power
    output suitable for the motors to be used in the setMotors function if in
    user assist mode.

        @see irPID
        @see setMotors
*/
void irToMotor(void) {
        
    if(!UAFlag) {
        IR_power.left = 3 * (TURNING_S*(TURNING_R-WHEELDIST/2) /
            (TURNING_R+WHEELDIST/2) - IR_PID.current_desired_variable);
        IR_power.right = 2.5 * (TURNING_S + IR_PID.current_desired_variable);
    }
    else {
        IR_power.right = 3 * (TURNING_S*(TURNING_R-WHEELDIST/2) /
            (TURNING_R+WHEELDIST/2) - IR_PID.current_desired_variable);
        IR_power.left = 2.5 * (TURNING_S + IR_PID.current_desired_variable);
    }  
}

/**
    irCalc acquires IR readings with the irGetMean function, determines
    which IR sensor has detected an object and calculates the appropriate IR
    PID values in the irPID function. The values are then converted to a power
    output suitable for the motors in the irToMotor function.

        @see irGetMean
        @see irPID
        @see irToMotor
*/
void irCalc(void){
    irReadValues();
    if(!irBufferRightHead){
        readings = irGetMean();
        if(readings.left > readings.right && readings.right<150){
            UAFlag = 1;     // turn right = 1
            mode = 0;
            IR_PID.IR = readings.right;
            irPID();
        } else if(readings.right>readings.left && readings.left<150) {
            UAFlag = 0;     // turn left = 0
            mode = 0;
            IR_PID.IR = readings.left;
            irPID();
        } else if(readings.right>100 && readings.left>100){
            mode = 1;
        }
        irToMotor();
    }   
}

/**
    userAssist checks to see if running in user assist mode, runs the
    necessary IR calculations in the irCalc function and resets Timer1.

        @see irCalc
*/
void userAssist(void) {
    if(INTCONbits.TMR0IF){
        if(config.mode == 2){
            irCalc();
            T0CONbits.TMR0ON = 0;       // Turn off timer

            sample_multiplier = config.IR_rate;
            sample_rate = 65535 - sample_multiplier * IR_MIN_SAMPLE;
            sample_rate_H = sample_rate >> 8;
            sample_rate_L = sample_rate & 0xFF;

            TMR0H = sample_rate_H;
            TMR0L = sample_rate_L;
            
            T0CONbits.TMR0ON = 1;       // Turn timer on

            INTCONbits.TMR0IF = 0;
        } 
    }
}

/**
    configInit sets the default values of the configuration settings for 
    normal and factory modes.
*/
void configInit(void) {
    config.pid_p = 1;
    config.pid_i = 1;
    config.pid_d = 1;
    config.yaw = 50; 
    config.IR_rate = 23;
    config.IR_samples = 10;
    config.max_speed = 100;
    IR_PID.desired_value = 80;
    config.IR_samples = 40; 
}
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Encoder variables and functions
// Macros for encoders
#define ENCODER_SAMPLE_RATE 0.0001408   // Based on Timer1
#define CONVERT_TO_RPM 0.0213           // Convert edges/second to RPM
#define NO_OF_SAMPLES 710               // Number of samples per RPM calc
#define MAX_RPM 80                      // Maximum RPM of motors

// Encoder global variables
unsigned char pinAR = 0;        // pinA right encoder
unsigned char pinBR = 0;        // pinB right encoder
unsigned char pinAL = 0;        // pinA left encoder
unsigned char pinBL = 0;        // pinB left encoder

unsigned char pinAROld = 0;     // previous pinA value right encoder
unsigned char pinALOld = 0;     // previous pinA value left encoder

unsigned short int edgeCountR = 0;      // Edges counted on right encoder
unsigned short int edgeCountL = 0;      // Edges counted on left encoder

unsigned short int overflowCount = 0;   // Timer overflow counter

float dt = ENCODER_SAMPLE_RATE * NO_OF_SAMPLES;     // Time for RPM calc

unsigned char rpmR = 0;        // RPM right encoder
unsigned char rpmL = 0;        // RPM left encoder

// Motor PID global variables
char encoderPowerR = 0;       // Encoder power for motor input
char encoderPowerL = 0;       // Encoder power for motor input

float powerErrorR = 0;        // Error between encoder and motor power 
float powerErrorL = 0;
        
float powerPIDR = 0;        // Output from PID control
float powerPIDL = 0;

float kp = 0.001;             // Initial motor PID proportional gain

/**
    lowVectorInterrupt is used to declare the lowPriorityISR Interrupt Service
    Routine and declare the ISR in the correct place in memory. This function
    must be called for the lowPriorityISR to function correctly.

        @see lowPriorityISR
*/
void lowPriorityISR(void);
#pragma code lowPriAddr=0x18        // Low priority vector at 0x18
void lowVectorInterrupt(void){
    _asm GOTO lowPriorityISR _endasm
}
#pragma code

/**
    lowPriorityISR is the Interrupt Service Routine that triggers whenever
    Timer1 overflows at a sampling frequency that is twice the fastest
    encoder rising edge frequency. The values from the encoder pins are read
    and if a rising edge has occured since the last sample the edge count for
    the appropriate encoder is increased. The current pinA status is saved as
    the previous pinA status for the next sample and the overflow counter is
    incremented so that the calculateRPM function can calculate the RPM after
    the total sample time has occurred. Timer1 is also reset to the initial
    values.

        @see calculateRPM
*/
#pragma interrupt lowPriorityISR
void lowPriorityISR(void) {
    INTCONbits.GIEL = 0;        // Disable low priority interrupts
    
    if (PIR1bits.TMR1IF) {
        //// When timer 1 overflows, store values from encoder
        // Right motor
        pinAR = PORTDbits.RD0;
        pinBR = PORTDbits.RD5;
        
        // Left motor
        pinAL = PORTDbits.RD7;
        pinBL = PORTDbits.RD6;
        
        // Compare old pinA values to new values, if LOW changes to
        // HIGH (rising edge) increment edgeCount variables
        // Right motor
        // Check for rising edge
        if (pinAR == 1 && pinAROld == 0) {
            edgeCountR++;
        }

        // Left motor
        // Check for rising edge
        if (pinAL == 1 && pinALOld == 0) {
            edgeCountL++;
        }       
        
        // Save current variables as old variables for next overflow
        pinAROld = pinAR;
        pinALOld = pinAL;
        
        // Increment overflow counter
        overflowCount++;
        
        // Reset Timer1
        TMR1H = 0xFF;
        TMR1L = 0xD3;
        
        // Reset Timer1 Overflow interrupt bit
        PIR1bits.TMR1IF = 0;
    }
    
    INTCONbits.GIEL = 1;        // Enable low priority interrupts
}
#pragma code

/**
    TMR1Setup sets up Timer1 as a 16-bit timer with a prescale of 1:8 and an
    intial delay of 0.0001408 seconds as a sample rate. Timer1 is started at
    the end of TMR1Setup to begin sampling.
*/
void TMR1Setup() {
    T1CONbits.RD16 = 1;         // Timer1 as 16-bit timer
    T1CONbits.T1CKPS1 = 1;      // Timer1 prescale of 8:1
    T1CONbits.T1CKPS0 = 1;
    T1CONbits.T1OSCEN = 0;      // Timer1 oscillator shut off
    T1CONbits.TMR1CS = 0;       // Use internal clock Fosc/4
    T1CONbits.TMR1ON = 0;       // Timer off
    
    // Sample rate
    TMR1H = 0xFF;               // 0xFFD3 gives a delay of 0.0001408 seconds
    TMR1L = 0xD3;               // at 8:1 prescale to use as a sample rate
    
    T1CONbits.TMR1ON = 1;       // Start the timer
    return;
}

/**
    PORTDSetup sets up RD0 and RD7 as input for the encoder A pins for the
    right and left encoders, respectively. RD5 and RD6 are set up as input
    for encoder B pins on right and left encoders, respectively. The ports
    are cleared to start with.
*/
void PORTDSetup() {
    TRISDbits.RD0 = 1;          // Input pinAR on RD0
    TRISDbits.RD7 = 1;          // Input pinAL on RD7
    
    TRISDbits.RD5 = 1;          // Input pinBR on RD5
    TRISDbits.RD6 = 1;          // Input pinBL on RD6

    PORTDbits.RD0 = 0;          // Clear ports to start with
    PORTDbits.RD7 = 0;
    PORTDbits.RD5 = 0;
    PORTDbits.RD6 = 0;

   
    return;
}

/**
    interruptSetup configures Timer1 as a low priority interrupt for the 
    TMR1IF to be used in the lowPriorityISR function when Timer1 overflows.

        @see lowPriorityISR
*/
void interruptSetup() {
    INTCONbits.GIEH = 0;        // Disables all interrupts
    
    RCONbits.IPEN = 1;          // Enable priority interrupts
    
    PIE1bits.TMR1IE = 1;        // Enable TMR1 overflow interrupt
    IPR1bits.TMR1IP = 0;        // Low priority interrupt
    
    INTCONbits.GIEH = 1;        // Enable high priority interrupts
    INTCONbits.GIEL = 1;        // Enable low priority interrupts
    
    return;
}

/**
    calculateRPM calculates the RPM of each encoder by dividing the encoder
    edge count determined in the lowPriorityISR function by the amount of time
    that has passed and multiplying by a constant CONVERT_TO_RPM to convert
    the edge per second reading to RPM.
    The RPM is converted to a power output suitable for the motors to be used
    in the encoderPIDCalculation function.
    The overflow and edge count variables used are also reset.

        @see lowPriorityISR
        @see CONVERT_TO_RPM
        @see encoderPIDCalculation
*/
void calculateRPM(void) {
    if (overflowCount >= NO_OF_SAMPLES) {
        // Right motor
        edgeCountR = edgeCountR * 1.067;
        edgeCountL = edgeCountL * 1.067;

        // Calculate RPM from edgeCount and dt
        rpmR = edgeCountR / dt * CONVERT_TO_RPM;
        if (rpmR > 80) {
            rpmR = 80;
        }
        
        // Left motor
        rpmL = edgeCountL / dt * CONVERT_TO_RPM;
        if (rpmL > 80) {
            rpmL = 80;
        }
        
        // Convert to power_reading for PID control
        // 
        encoderPowerR = rpmR * 1.5875;
        encoderPowerL = rpmL * 1.5875;

        // Reset count variables
        overflowCount = 0;
        edgeCountR = 0;
        edgeCountL = 0;
    }
    // End of encoder variables
}

/**
    correctEncoderDirection corrects the encoder direction based on the 
    power input from the motors.
*/
void correctEncoderDirection(void) {
    if (power.right < 0) {
        encoderPowerR = encoderPowerR * -1;
    }

    if (power.left < 0) {
        encoderPowerL = encoderPowerL * -1;
    }
}

/**
    encoderPIDCalculation uses proportional control to adjust the power that
    is output to the motors by the setMotors function.

        @see setMotors
*/
void encoderPIDCalculation(void) {
    // Set pid gains from controller
    kp = config.pid_p * 0.001;
    
    powerErrorR = power.right - encoderPowerR;
    powerErrorL = power.left - encoderPowerL;

    powerPIDR = kp * powerErrorR;
    powerPIDL = kp * powerErrorL;


    power.right = power.right + (char)powerPIDR;
    power.left = power.left + (char)powerPIDL;
    
    return;
}
//////////////////////////////////////////////////////////////////////////////

void main(void){
   
    // Setup for USART
    configureUART();
    configureUARTInterrupts();
    
	motorsSetup();
    max_speed = 80;
    max_yaw = 300;
    configInit();
    mode = 1;

    // Setup for encoders
    TMR1Setup();
    PORTDSetup();
    interruptSetup();
    // End setup for encoders
    
    // Setup for IR timer
    TMR0Setup();
    initADC();


    for(;;){
        
       // Send string back to controller
        xBeeSend("SSending");
        power = vectorToMotor(velocity);// output x,y value 
        
        userAssist();
        
        // Start Encoder RPM calcualtion
        // Calculate rpm from encoders
        calculateRPM();

        if(velocity.z == 1){
            mode = 1;
        }
        // Correct encoder direction
        correctEncoderDirection();
        
        // End encoder RPM calculations
        
        // Encoder PID calculation
        encoderPIDCalculation();
        
        // Decipher what mode system is in and set power accordingly
        if(mode)setMotors(power);
        if(!mode)setMotors(IR_power);
        
    }
}