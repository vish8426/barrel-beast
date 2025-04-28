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
#define FRAME_BUFFER_SIZE 6

// Macros for encoders
#define ENCODER_SAMPLE_RATE 0.0001408       // Based on Timer1
#define CONVERT_TO_RPM 0.0213
#define NO_OF_SAMPLES 710
#define MAX_RPM 80

// IR 
#define IR_BUFFER_MAX 40
#define WHEELDIST 200
#define TURNING_R 220
#define TURNING_S 50
#define IR_MIN_SAMPLE 655

unsigned char sample_multiplier = 1;
unsigned short int sample_rate = 0;
unsigned char sample_rate_H = 0;
unsigned char sample_rate_L = 0;

void configureUART(void);
void configureUARTInterrupts(void);
void XBeeRead(void);
void saveJoyStickValues(void);
void xBeeSend(char * string);
void highPriorityISR(void);
void xBeeOnReceive(void);
void delay(unsigned int itime);
void saveConfigValues(void);
void xBeeSendDisplay(void);
void irReadValues(void);
void initADC(void);
unsigned char analogRead(char channel);
void irTrigger(void);
void timer0setup(void);
void IRInterrupt(void);
void lowPriorityISR(void);
void config_init(void);
void pid_calculation(void);

// Encoder function declarations
void low_vector_interrupt(void);
void low_isr(void);

void TMR1_setup(void);          // Run the timer
void PORTD_setup(void);         // Read pinB values on PORTD
void interrupt_setup(void);     // Set up interrupts
void calculate_rpm(void);       // Calculate RPM values

unsigned char irBufferLeft[IR_BUFFER_MAX];
unsigned char irBufferRight[IR_BUFFER_MAX];

unsigned char irBufferLeftHead = 0;
unsigned char irBufferRightHead = 0;


// Struct to store PID variables for use in calculations
typedef struct {
    char desired_value;
    char IR;
    char last_error;
    float current_desired_variable;
    char error;
    float integral;
    float derivative;
}PIDvariables;
PIDvariables IR_PID;

// User assist globals
char UAFlag = 0;
char IR_turn = 0; // 0 == LEFT, 1 == RIGHT
char IRFlag;
char mode = 1;
int iterations = 0;

// Encoder variables
// Pin input variables
unsigned char pinA_R = 0;
unsigned char pinB_R = 0;
unsigned char pinA_L = 0;
unsigned char pinB_L = 0;

// Previous pinA variables
unsigned char pinA_R_old = 0;
unsigned char pinA_L_old = 0;

// Edge counters
unsigned short int edge_count_R = 0;
unsigned short int edge_count_L = 0;

// Overflow counter
unsigned short int overflow_count = 0;

// Doubles for calculations
float dt = ENCODER_SAMPLE_RATE * NO_OF_SAMPLES;
// Could make rpm variables into ints
unsigned char rpm_R = 0;
unsigned char rpm_L = 0;

char encoder_power_R = 0;
char encoder_power_L = 0;
// End Encoder variables

////////PID
// PID function and variable declaration
float power_error_R = 0;
float power_error_L = 0;
        
float power_pid_R = 0;
float power_pid_L = 0;

float kp = 0.1;
float ki = 0.0;
float kd = 0.0;

////// End PID functions/variables

float speed; 
float speedscale;
float max_speed;
unsigned int max_yaw = 100;
float yawscale;
 
// Struct to store left and right IR values
typedef struct{
    unsigned char left;
    unsigned char right;
} IRResult;
IRResult mean;
IRResult irGetVariance(void);
IRResult irGetMean(void);
IRResult readings;


// XBee Globals
int i = 0;
char readValues[4];
int startReading = FALSE;

// Struct to store values from configuration menu input
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

// Struct to store left and right power values for motor
typedef struct
{
    char left;
    char right;
} Motor;
Motor power;
Motor IR_power;

void IR_To_Motor(void);

// Struct to store joystick values
typedef struct{
    signed char x;
    signed char y;
    char z;
}Vector;
Vector velocity;

Motor vectorToMotor(Vector v);

Motor vectorToMotor(Vector v){
    Vector output;
    Motor motor_out;
    int xadjust; 
    int yadjust;
    
        
    // scale v.x to a percentage according to max yaw/ 100 
    xadjust = v.x;
    xadjust = xadjust *config.yaw/100; 
    v.x = -xadjust; 
    
    yadjust = v.y;
    yadjust = yadjust * config.max_speed/100;
    v.y = yadjust;
    
    // end max yaw 

     if (v.x == 0)
    {
        output.x = v.x;
        output.y = v.y;
    }
    else
    {
        if((fabs(v.x)/fabs(v.y))>1){

            output.x = (fabs(v.x) * v.x) / (fabs(v.x) + fabs(v.y));
            output.y = (fabs(v.x) * v.y) / (fabs(v.x) + fabs(v.y));

        }
        else{
            output.x = (fabs(v.y) * v.x) / (fabs(v.x) + fabs(v.y));
            output.y = (fabs(v.y) * v.y) / (fabs(v.x) + fabs(v.y));
        }
    }

    motor_out.left = output.y + output.x;
    motor_out.right = output.y - output.x;

    return motor_out;
}


// Function to configure UART on the Robot side
void configureUART(void){
    
    TXSTAbits.BRGH = 1; // High speed mode (y=16)
    SPBRG = 64;         // X = (10^7/9600*16) - 1 = 64.1
    TXSTAbits.SYNC = 0; // Asynchronous mode
    RCSTAbits.SPEN = 1; // Enable serial port
   
    TXSTAbits.TX9 = 0;  // Enable 8 bit transmission
    TXSTAbits.TXEN = 1; // Enable transmit
    RCSTAbits.RX9 = 0;  // 9 bit receive
    RCSTAbits.CREN = 1; // receive enabled
    
    TRISCbits.RC6 = 0;  // Enable Pin 6 as Tx
    TRISCbits.RC7 = 1;  // Enable pin 7 as Rx
    
}   

// Function to configure UART interrupts on Robot side
void configureUARTInterrupts(void){
   
    INTCONbits.GIEH = 0; // Disable Interrupts
    INTCONbits.GIEL = 0; // Disable Peripheral interrupts 
    RCONbits.IPEN = 1;   // Enable Priority
    PIE1bits.RCIE = 1;   // Enable USART Receive Interrupt
    IPR1bits.RCIP = 1;   // Enable High Priority on receive interrupt *
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1; 
    INTCONbits.PEIE = 1; // Re-enable interrupts
   
}

void setMotors(Motor power){
    char left;
    char right;
    if (power.left<0){

        // make positive
        left = power.left*-1;

       //set INA to 0 and INB to 1 to reverse
       PORTDbits.RD3= 0;
       PORTDbits.RD4 =1;


    }else{

        // make positive
        left = power.left;
        //set INA to 0 and INB to 1 to reverse
       PORTDbits.RD3= 1;
       PORTDbits.RD4 =0;
    }if (power.right<0){

        // make positive
        right = power.right*-1;

       //set INA to 0 and INB to 1 to reverse
       PORTDbits.RD1 =0;
       PORTDbits.RD2 =1;


    }else{

        // make positive
        right = power.right;
        //set INA to 0 and INB to 1 to reverse    
        PORTDbits.RD1= 1;
        PORTDbits.RD2 =0;
    }

    //left=left/4;
    //right=right/4;

    //changed to use the LSB automatically with less code
    CCP1CONbits.DC1B =left;
    CCP2CONbits.DC2B =right;

    //left = left >> 2;
    //right = right >> 2;

    CCPR1L = left; //left motor output to RC1

    CCPR2L = right; //Right motor output to RB3
}

void motorsSetup(){
    TRISD = 0x00; //set D all outputs for LED
    
    TRISCbits.RC1 = 0;
    TRISCbits.RC2 =0;
    TRISCbits.CCP2 =1;
    TRISBbits.CCP2 =1;
    TRISBbits.RB3 = 0;


    //timer 2 on and prescale 1 might need 11 for 16 prescale
    T2CONbits.TMR2ON =1;
    T2CONbits.T2CKPS0 =1;
    T2CONbits.T2CKPS1 =0;


    // PWM setup
    CCP1CONbits.CCP1M3 =1;
    CCP1CONbits.CCP1M2 =1;

    CCP2CONbits.CCP2M3 =1;
    CCP2CONbits.CCP2M2 =1;
    CCP2CONbits.CCP2M1 =1;
    CCP2CONbits.CCP2M0 =1;


    // make PR2 to 127 for 1.2khz pwm duty to fit announcement
    PR2 = 0x7F;

    
}

// Function to send strings from Robot to Commander over XBee Communication
void xBeeSend(char * string){
    char c;
    
    // Loop through all characters in string
    while(c = *(string ++)){
        while(PIR1bits.TXIF == 0);     // Wait till transmit buffer is clear
        TXREG = c;                     // Transmit character over USART
    }

}

// Interrupt Initialisation for USART receive
#pragma code highPriAddr = 0x08
void xBeeOnReceive(void){
    _asm goto highPriorityISR _endasm
}
#pragma code

// ISR for USART Receive Protocol
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

    // If start reading flag is off dont read character. This stops errors in transmission effecting output variables
    if(startReading == FALSE)return;
    
    // Save current character into read buffer
    readValues[i] = currentChar;
    i++;

    // If 3 characters have been read after stop bit
    if(i == 3){

        // If start bit was V joystick values were sent and save buffer into velocity struct
        if(readValues[0] == 'V'){
            saveJoyStickValues(); 

        }      
        
        // If start bit was C config values were sent and save buffer into config array
        if(readValues[0] == 'C'){
            saveConfigValues();
        }
        
        // Reset start reading flag and index
        startReading = FALSE;
        i = 0;     
    }
}
#pragma code

void initADC(void){
    TRISA  = 0xff;            // PORT A Set as Input Port
}

void irReadValues(void){
    unsigned char right = analogRead(2); 
    unsigned char left = analogRead(3);

    irBufferLeftHead =  ( irBufferLeftHead +  1 ) % config.IR_samples;
    irBufferRightHead = ( irBufferRightHead + 1 ) % config.IR_samples;
    
    irBufferLeft[irBufferLeftHead] = left;
    irBufferRight[irBufferRightHead] = right;
}

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

IRResult irGetVariance(void) {
    unsigned char i;
    IRResult var;

    unsigned long leftVar = 0;
    unsigned long rightVar = 0;
    IRResult mean = irGetMean();
    
    for (i = 0; i < config.IR_samples; i++){
        leftVar += (irBufferLeft[i] - mean.left) * (irBufferLeft[i] - mean.left);
        rightVar += (irBufferRight[i] - mean.right) * (irBufferRight[i] - mean.right);
    }
    var.left = leftVar/config.IR_samples/256;
    var.right = rightVar/config.IR_samples/256;
    
    return var;
}

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

void irTrigger(void) {
    IRResult temporary;
    irReadValues();
    
    temporary = irGetMean();
    if(temporary.left > temporary.right) {
        if(temporary.left > 40 && temporary.left < 100) { // too big range??
            UAFlag = 1;
            IR_turn = 0;
        }
    }
    else {
        if(temporary.right > 40 && temporary.right < 100) {
            UAFlag = 1;
            IR_turn = 1;
        }
    }
    
} 


void timer0setup(void) { // Delays the function by the given Sample Rate, 2.5 MHz
    T0CONbits.TMR0ON = 0;       // Start timer off
    T0CONbits.T08BIT = 0;       // 16-bit
    T0CONbits.T0CS = 0;         // internal clock
    T0CONbits.PSA = 0;          // use prescaler
    T0CONbits.T0PS2 = 0;        // 1:64 prescaler
    T0CONbits.T0PS1 = 0;
    T0CONbits.T0PS0 = 0;
    
    sample_multiplier = config.IR_rate;
    sample_rate = 65535 - sample_multiplier * IR_MIN_SAMPLE;
    sample_rate_H = sample_rate >> 8;
    sample_rate_L = sample_rate & 0xFF;
    
    TMR0H = sample_rate_H;
    TMR0L = sample_rate_L;
    
    T0CONbits.TMR0ON = 1;        // TUrn timer on
}


#pragma code low_vector=0x18        // Low priority vector at 0x18
void low_vector_interrupt(void){
    _asm GOTO low_isr _endasm
}
#pragma code

#pragma interrupt low_isr
void low_isr(void) {
    INTCONbits.GIEL = 0;        // Disable low priority interrupts
    
    if (PIR1bits.TMR1IF) {
        //// When timer 1 overflows, store values from encoder
        // Right motor
        pinA_R = PORTDbits.RD0;
        pinB_R = PORTDbits.RD5;
        
        // Left motor
        pinA_L = PORTDbits.RD7;
        pinB_L = PORTDbits.RD6;
        
        //// Compare old pinA values to new values, if LOW changes to
        ////    HIGH (rising edge) increment edge_count variables
        ////    Only after overflow has occurred once already
        //// Then check and set direction variables
        // Right motor
        // Check for rising edge
        if (pinA_R == 1 && pinA_R_old == 0) {
            edge_count_R++;
        }

        // Left motor
        // Check for rising edge
        if (pinA_L == 1 && pinA_L_old == 0) {
            edge_count_L++;
        }       

        
        // Save new variables as old variables for next overflow
        pinA_R_old = pinA_R;
        pinA_L_old = pinA_L;
        
        // Increment overflow counter
        overflow_count++;
        
        // Reset Timer1
        TMR1H = 0xFF;
        TMR1L = 0xD3;
        
        // Reset Timer1 Overflow interrupt bit
        PIR1bits.TMR1IF = 0;
    }
    
    INTCONbits.GIEL = 1;        // Enable low priority interrupts
}
#pragma code
    


void IR_To_Motor(void) {
        
    if(!UAFlag) {
        IR_power.left = 3 * (TURNING_S*(TURNING_R-WHEELDIST/2)/(TURNING_R+WHEELDIST/2) - IR_PID.current_desired_variable);
        IR_power.right = 2.5 * (TURNING_S + IR_PID.current_desired_variable);
    }
    else {
        IR_power.right = 3 * (TURNING_S*(TURNING_R-WHEELDIST/2)/(TURNING_R+WHEELDIST/2) - IR_PID.current_desired_variable);
        IR_power.left = 2.5 * (TURNING_S + IR_PID.current_desired_variable);
    }  
 }
    

void pid(void){
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
      IR_PID.current_desired_variable = (kp_ir*IR_PID.error) + (ki_ir*IR_PID.integral) + (kd_ir*IR_PID.derivative);

   // Store previous error
      IR_PID.last_error = IR_PID.error;
}



// Function to save values from read buffer into velocity struct
void saveJoyStickValues(void){
    velocity.x = readValues[1];
    velocity.y = readValues[2];
    velocity.z = readValues[3];
}

// Function to save values from read buffer into configuration union array
void saveConfigValues(void){
    config.array[readValues[1]] = readValues[2];
}

void IRCalc(void){
    irReadValues();
    if(!irBufferRightHead){
        readings = irGetMean();
        if(readings.left > readings.right && readings.right<150){
            UAFlag = 1;     // turn right = 1
            mode = 0;
            IR_PID.IR = readings.right;
            pid();
        } else if(readings.right>readings.left && readings.left<150) {
            UAFlag = 0;     // turn left = 0
            mode = 0;
            IR_PID.IR = readings.left;
            pid();
        } else if(readings.right>100 && readings.left>100){
            mode = 1;
        }
        IR_To_Motor();
    }   
}

void main(void){
   
    // Setup for USART
    configureUART();
    configureUARTInterrupts();
    
	motorsSetup();
    max_speed = 80;
    max_yaw = 300;
    IRFlag = 0;
    config_init();
    mode = 1;

    // Setup for encoders
    TMR1_setup();
    PORTD_setup();
    timer0setup();
    interrupt_setup();
    // End setup for encoders
    
    for(;;){
        
       // Send string back to controller
        xBeeSend("SSending");
        power = vectorToMotor(velocity);// output x,y value 
        
        if(INTCONbits.TMR0IF){
            if(config.mode == 2){
                IRCalc();
                T0CONbits.TMR0ON = 0;       // TUrn off timer

                sample_multiplier = config.IR_rate;
                sample_rate = 65535 - sample_multiplier * IR_MIN_SAMPLE;
                sample_rate_H = sample_rate >> 8;
                sample_rate_L = sample_rate & 0xFF;

                TMR0H = sample_rate_H;
                TMR0L = sample_rate_L;
                
                T0CONbits.TMR0ON = 1;       // TUrn timer on

                INTCONbits.TMR0IF = 0;
            } 
        }
        // Start Encoder RPM calcualtion
        // Calculate rpm from encoders
        calculate_rpm();

        if(velocity.z == 1){
            mode = 1;
        }
        // Correct encoder direction
        if (power.right < 0) {
            encoder_power_R = encoder_power_R * -1;
        }

        if (power.left < 0) {
            encoder_power_L = encoder_power_L * -1;
        }
        // End encoder RPM calculations
        
        pid_calculation();
        
        // Decipher what mode system is in and set power accordingly
        if(mode)setMotors(power);
        if(!mode)setMotors(IR_power);
        
    }
}

void delay (unsigned int itime){

    int i, j; 
    
    for(i=0;i<itime;i++){
        for(j=0; j<135; j++);
    }
    
}

// Encoder function definitions
void TMR1_setup() {
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

void PORTD_setup() {
    TRISDbits.RD0 = 1;          // Input pinA_R on RD0
    TRISDbits.RD7 = 1;          // Input pinA_L on RD7
    
    TRISDbits.RD5 = 1;          // Input pinB_R on RD5
    TRISDbits.RD6 = 1;          // Input pinB_L on RD6

    PORTDbits.RD0 = 0;          // Clear ports to start with
    PORTDbits.RD7 = 0;
    PORTDbits.RD5 = 0;
    PORTDbits.RD6 = 0;

   
    return;
}

void interrupt_setup() {
    // Use Timer1 overflow interrupt as a sample rate to calculate RPM
    INTCONbits.GIEH = 0;        // Disables all interrupts
    
    RCONbits.IPEN = 1;          // Enable priority interrupts
    
    PIE1bits.TMR1IE = 1;        // Enable TMR1 overflow interrupt
    IPR1bits.TMR1IP = 0;        // Low priority interrupt
    
    INTCONbits.GIEH = 1;        // Enable high priority interrupts
    INTCONbits.GIEL = 1;        // Enable low priority interrupts
    
    return;
}

void calculate_rpm(void) {
    if (overflow_count >= NO_OF_SAMPLES) {
        // Right motor
        edge_count_R = edge_count_R * 1.067;
        edge_count_L = edge_count_L * 1.067;

        // Calculate RPM from edge_count and dt
        rpm_R = edge_count_R / dt * CONVERT_TO_RPM;
        if (rpm_R > 80) {
            rpm_R = 80;
        }
        
        // Left motor
        rpm_L = edge_count_L / dt * CONVERT_TO_RPM;
        if (rpm_L > 80) {
            rpm_L = 80;
        }
        
        // Convert to power_reading for PID control
        // 
        encoder_power_R = rpm_R * 1.5875;
        encoder_power_L = rpm_L * 1.5875;

        // Reset count variables
        overflow_count = 0;
        edge_count_R = 0;
        edge_count_L = 0;
    }
    // End of encoder variables
}

void pid_calculation(void) {
    // Set pid gains from controller
    kp = config.pid_p * 0;
    ki = config.pid_i * 0;
    kd = config.pid_d * 0;
    
    power_error_R = power.right - encoder_power_R;
    power_error_L = power.left - encoder_power_L;

    power_pid_R = kp * power_error_R;
    power_pid_L = kp * power_error_L;


    power.right = power.right + (char)power_pid_R;
    power.left = power.left + (char)power_pid_L;
    
    return;
}

// Function to initialize configuration values if not set by user in menus
void config_init(void) {
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