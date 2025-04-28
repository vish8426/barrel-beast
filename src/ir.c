// IR Macros
#define IR_BUFFER_MAX 40            // Maximum buffer size for IR samples
#define WHEELDIST 200               // Wheelbase in mm
#define TURNING_R 220               // Turning radius
#define TURNING_S 50                // Turning speed
#define IR_MIN_SAMPLE 655           // Minimum number of IR samples

// IR global variables
unsigned char sample_multiplier = 1;        // Multiplier for configuring sample
                                            // rate
unsigned short int sample_rate = 0;         // IR sample rate
unsigned char sample_rate_H = 0;            // For high byte TMR0H
unsigned char sample_rate_L = 0;            // For low byte TMR0L

unsigned char irBufferLeft[IR_BUFFER_MAX];  // Buffer for left IR values
unsigned char irBufferRight[IR_BUFFER_MAX]; // Buffer for right IR values

unsigned char irBufferLeftHead = 0;         // Buffer for reading left IR value
unsigned char irBufferRightHead = 0;        // Buffer for reading right IR value

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
    TMR0Setup sets up Timer1 as a 16-bit timer with a prescale of 1:64 and an
    initial delay dependent on the input IR sample rate from the commander.
    Timer0 is started at the end of TMR0Setup to begin sampling.
*/
void TMR0Setup(void) { // Delays the function by the given Sample Rate, 2.5 MHz
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
    userAssist checks to see if running in user assist mode, runs the necessary 
    IR calculations in the irCalc function and resets Timer1.

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