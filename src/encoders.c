// Macros for encoders
#define ENCODER_SAMPLE_RATE 0.0001408   // Based on Timer1
#define CONVERT_TO_RPM 0.0213           // Convert edges/second to RPM
#define NO_OF_SAMPLES 710               // Number of samples per RPM calculation
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

float dt = ENCODER_SAMPLE_RATE * NO_OF_SAMPLES;     // Time for RPM calculation

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
    encoderPIDCalculation uses proportional control to adjust the power that is
    output to the motors by the setMotors function.

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