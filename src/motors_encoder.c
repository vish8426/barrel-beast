#include <stdio.h>
#include <stdlib.h>
#include "ConfigRegsPIC18F452.h"
#include <p18f452.h>
#include <string.h>
#include <usart.h>
#include <math.h>
#define TRUE 1 
#define FALSE 0 
#define FRAME_BUFFER_SIZE 6

// Macros for encoders
#define ENCODER_SAMPLE_RATE 0.0001408       // Based on Timer1
#define CONVERT_TO_RPM 0.0213
#define NO_OF_SAMPLES 710


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

// Encoder function declarations
void low_vector_interrupt(void);
void low_isr(void);

void TMR1_setup(void);          // Run the timer
void PORTD_setup(void);         // Read pinB values on PORTD
void interrupt_setup(void);     // Set up interrupts
void calculate_rpm(void);       // Calculate RPM values

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
float rpm_R = 0;
float rpm_L = 0;
// End Encoder variables




float speed; 
float speedscale;
float max_speed;


int i = 0;
char readValues[4];
int startReading = FALSE;

char FrameBuffer[FRAME_BUFFER_SIZE];
unsigned char FrameHead = 0;
unsigned char NewCmdFlag = 0;


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
        
        
    };
    
    unsigned char array[8];
}ConfigValues;

ConfigValues config;

typedef struct
{
    char left;
    char right;
} Motor;
Motor power;

typedef struct{
    signed char x;
    signed char y;
    char z;
}Vector;
Vector velocity;

union{
    struct{
        char V;
        Vector vector;
        char z2;
    };
    struct{
        char S;
        unsigned char mu1;
        unsigned char var1;
        unsigned char mu2;
        unsigned char var2;
    };
    struct{
        char D;
        char mpwl;
        char mpwr;
        char spr1;
        char spr2;
    };
    struct{
        char C;
        unsigned char index;
        unsigned char value;
        char spr3;
        char spr4;
    };
    char array[FRAME_BUFFER_SIZE - 1];
} XBeeCmd;


Motor vectorToMotor(Vector v);

Motor vectorToMotor(Vector v)
{
    Vector output;
    Motor motor_out;

   // x=x-127;
    //y=y-127;

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


void configureUART(void){
    
    TXSTAbits.BRGH = 1; // Low speed mode (y=16)
    SPBRG = 64;          // X = (10^7/9600*16) - 1 = 64.1
    TXSTAbits.SYNC = 0; // Asynchronous mode
    RCSTAbits.SPEN = 1; // Enable serial port
   
    TXSTAbits.TX9 = 0;  // Enable 8 bit transmission
    TXSTAbits.TXEN = 1; // Enable transmit
    RCSTAbits.RX9 = 0;  // 9 bit receive
    RCSTAbits.CREN = 1; // receive enabled
    
    TRISCbits.RC6 = 0;  // Enable Pin 6 as Tx
    TRISCbits.RC7 = 1;  // Enable pin 7 as Rx
    
}   


void configureUARTInterrupts(void){
   
    INTCONbits.GIEH = 0; // Disable Interrupts
    INTCONbits.GIEL = 0; // Disable Peripheral interrupts 
    RCONbits.IPEN = 1;   // Enable Priority
    PIE1bits.RCIE = 1;   // Enable USART Receive Interrupt
    IPR1bits.RCIP = 1;   // Enable High Priority on receive interrupt *
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1; // Re-enable interrupts
    INTCONbits.PEIE = 1;
   
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


void xBeeSend(char * string){
    char c;
    while(c = *(string ++)){
        while(PIR1bits.TXIF == 0);
        TXREG = c;
    }

}

void xBeeWrite(char val){
    while(BusyUSART());
    WriteUSART(val);
}

    
void xBeeInitFrameBuffer(void){
    int i;
    for (i = 0; i < FRAME_BUFFER_SIZE; i++){
        FrameBuffer[i] = 0;
    }
}

void xBeeAddFrame(char byte){
   FrameHead = (FrameHead + 1) % FRAME_BUFFER_SIZE;
   FrameBuffer[FrameHead] = byte;
}

char xBeeFrameStatus(void){
    char start = FrameBuffer[ ( FrameHead + FRAME_BUFFER_SIZE - 1 ) % FRAME_BUFFER_SIZE ];
    char end = FrameBuffer[FrameHead];

    if (end != 0) return -1;
    if (start == 'V') return 'V';
    if (start == 'C') return 'C';
    if (start == 'D') return 'D';
    return -1;
}



char xBeeHasNewCmd(){
    if (NewCmdFlag){
        NewCmdFlag = 0;
        return 1;
    }
    return 0;
}

void xBeeTransmitCmd(void){
    int i;
    for (i = 0; i < FRAME_BUFFER_SIZE - 1; i++){
        xBeeWrite(XBeeCmd.array[i]);
    }
    xBeeWrite(0);
}

#pragma code highPriAddr = 0x08
void xBeeOnReceive(void){
    _asm goto highPriorityISR _endasm
}

#pragma code low_vector=0x18        // Low priority vector at 0x18
void low_vector_interrupt(void) {
    _asm GOTO low_isr _endasm
}

#pragma interrupt highPriorityISR
void highPriorityISR(void){
    
    int test;
    char byte = RCREG;
    xBeeAddFrame(byte);
    
    //If xBeeFrameStatus not -1 then transfer the frame to the XBeeCmd variable 
    if (xBeeFrameStatus() == -1) return;
    NewCmdFlag = 1;
    for (i = 0; i < FRAME_BUFFER_SIZE - 1; i++){
        XBeeCmd.array[i] = FrameBuffer[(i + FrameHead+FRAME_BUFFER_SIZE-1) % FRAME_BUFFER_SIZE];
    }
    test = 0;
     
}

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

void main(void){
    
    int frameBufferOverflow = 0;
    int status;
    configureUART();
    configureUARTInterrupts();
	motorsSetup();
    max_speed = 80;
    
    // Setup for encoders
    TMR1_setup();
    PORTD_setup();
    interrupt_setup();
    // End setup for encoders

    for(;;){
  
//        while(!xBeeHasNewCmd()){
//            frameBufferOverflow++;
//            if(frameBufferOverflow == 0){
//                status = 0;
//      
//            }      
//        }
//        
//        frameBufferOverflow = 0;
//        
//        if(XBeeCmd.V == 'V'){
//            velocity = XBeeCmd.vector;
//            XBeeCmd.D = 'D';
//            XBeeCmd.mpwl = 100;
//            xBeeTransmitCmd();
//        }
//        
//        if(XBeeCmd.C == 'C'){
//            config.array[XBeeCmd.index] = XBeeCmd.value;
//            if(!config.max_speed)config.max_speed = 80;
//        }
        
//        xBeeWrite("Sending");
        /*----------*/
        power = vectorToMotor(velocity);// output x,y value inputs
        
        speed = ((float)power.left + (float)power.right)/2 *0.625;
        max_speed = (float)config.max_speed;

        
        speedscale = max_speed * 0.0125;
        power.left = power.left * speedscale;
        power.right = power.right *speedscale;
       
        power.left = 96;
        power.right = 96;
        setMotors(power);
        
        if(status == 1){
            setMotors(power);
        }
        
        // Encoder rpm calculation
        calculate_rpm();
    }
}

void delay (unsigned int itime){

    int i, j; 
    
    for(i=0;i<itime;i++){
        for(j=0; j<135; j++);
    }
    
}

//    char currentChar = RCREG;
//    
//    //if(RCSTAbits.FERR ==  0 && RCSTAbits.OERR == 0){
//    if((currentChar == 'A'||currentChar == 'C') && startReading == FALSE){
//        i = 0;
//        startReading = TRUE;
//        readValues[i] = currentChar;
//        i++;
//        return;
//    }
//
//    if(startReading == FALSE)return;
//    readValues[i] = currentChar;
//    i++;
//
//    if(i == 3){
//
//        if(readValues[0] == 'A'){
//            saveJoyStickValues(); 
//            xBeeSendDisplay();
//
//        }      
//        if(readValues[0] == 'C'){
//            saveConfigValues();
//        }
//        startReading = FALSE;
//        i = 0;
//        

//
//void xBeeSendDisplay(void){
//    
//    int mean = 5 ;
//    int variance = 2;
//    char dispString[33];
//    config.display = 1;
//    
//    if(config.display == 1){  
//        xBeeSend('D');
//        xBeeSend(power.left);
//        xBeeSend(power.right);
//        xBeeSend('0');
//        xBeeSend('0');
//        xBeeSend(0);   
//    }
//    
////    if(config.display ==2){       
////        sprintf(dispString,"Power Right:%4cPower Left:%5c",power.right,power.left);
////        xBeeSend(dispString);      
////    }
//    
//}

    

//void saveConfigValues(void){    
//    int pos = readValues[1];
//    config.array[pos] = readValues[2];
//}

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

        // Left motor
        rpm_L = edge_count_L / dt * CONVERT_TO_RPM;

        // Reset count variables
        overflow_count = 0;
        edge_count_R = 0;
        edge_count_L = 0;
    }
}