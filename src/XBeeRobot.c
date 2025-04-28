/**
	configValues is a struct used to store configuration values entered by user
	on the controller menue interface
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
	Vector is a struct used to store joystick values once transmitted over USART and xBee communication
*/
typedef struct{
    signed char x;
    signed char y;
    char z;
}Vector;
Vector velocity;


// XBee Global Variables
int i = 0; 					// Index counter for receiving strings
char readValues[4]; 	    // Receive Character Buffer
int startReading = FALSE;   // Start Reading flag


/**
	configureUART sets the appropriate configuration bits inside the TXSTA and RCSTAbits
	registers in order for USART to function correctly as per project requirements. PORTC6 
	is set as an output to enable Transmission and PORTC7 is set as an input to enable receiving.
	This function must be called before receiving or transmitting strings via USART using xBeeSend and xBeeOnReceive
	
		@see xBeeOnReceive & xBeeSend


*/
void configureUART(void){
    
    TXSTAbits.BRGH = 1;  // High speed mode (y=16)
    SPBRG 		   = 64; // X = (10^7/9600*16) - 1 = 64.1
    TXSTAbits.SYNC = 0;  // Asynchronous mode
    RCSTAbits.SPEN = 1;  // Enable serial port
   
    TXSTAbits.TX9  = 0;  // Enable 8 bit transmission
    TXSTAbits.TXEN = 1;  // Enable transmit
    RCSTAbits.RX9  = 0;  // 8 bit receive
    RCSTAbits.CREN = 1;  // receive enabled
    
    TRISCbits.RC6  = 0;  // Enable Pin 6 as Tx
    TRISCbits.RC7  = 1;  // Enable pin 7 as Rx
    
} 

/**
	configureUARTInterrupts sets the appropriate configuration bits in order to enable interrupt on
	receive for the system. This function must be called before xBeeOnReceive.
		@see xBeeOnReceive

*/
void configureUARTInterrupts(void){
   
    INTCONbits.GIEH = 0; // Disable Interrupts
    INTCONbits.GIEL = 0; // Disable Peripheral interrupts 
    RCONbits.IPEN   = 1; // Enable Priority
    PIE1bits.RCIE   = 1; // Enable USART Receive Interrupt
    IPR1bits.RCIP   = 1; // Enable High Priority on receive interrupt *
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1; 
    INTCONbits.PEIE = 1; // Re-enable interrupts
   
}

/**
	xBeeSend is used to send strings from the robot to controller wirelessly over USART
	and xBee transmission
	
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
	xBeeOnReceive is used to declare the USART receive interrupt and declare the ISR
	in the correct place in memory. This function must be called for the highPriorityISR to function correcty
	
		@see highPriorityISR
*/
#pragma code highPriAddr = 0x08
void xBeeOnReceive(void){
    _asm goto highPriorityISR _endasm
}
#pragma code

/**
	highPriorityISR is the Interrupt Service Routine involved with recieving configuration values and storing them
	in the correct variables. This function uses saveConfigValues & saveConfigValues to store received strings correctly
	
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

/**
	saveJoyStickValues saves the x,y and z joystick values into the velocity struct if the 
	appropriate start bit has been read
*/
void saveJoyStickValues(void){
    velocity.x = readValues[1]; // Save buffer position 1 as x
    velocity.y = readValues[2]; // Save buffer position 2 as y
    velocity.z = readValues[3]; // Save buffer position 3 as z
}

/**
	saveJoyStickValues saves configuration values into the correct index position of the configuration
	array defined in the config union.
*/
void saveConfigValues(void){
    config.array[readValues[1]] = readValues[2]; // Save value in buffer position 2 into array position dictated by buffer position 1
}

/**
	config_init sets the default values of the configuration settings for normal
	and factory modes
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

