
/**********************************************************************************************************************
*   Rebeka Henry                                                                                                      *
*   Controlling the Servo Motor                                                                                       *
*   8/7/2020                                                                                                          *
*   The program below implements I2C using the Beaglebone Black and the PCA9685 Servo Controller.                     *
*   In order to set up I2C, the control module, I2C2, and CM_PER addresses are used to initialize                     *
*   the BBB as the Master Transmitting Device. This essentially entails turning on the clock, modules like GPIO1,     *
*	I2C2, TIMER2, pin muxing, prescaling the speed of the clock, etc.The sequence of steps follow the How to program  *             
*   I2C from the Sitara Manual. In order to transmit data to control the PCA9685, we use the configuration            *
*   register to enable I2C2 and make the BB the Master Transmitter. Before transmitting the data, we also             *
*   wait for the system status register on the bus to give us the signal to set the slave address and the             *   
*   data counter. The next step is to initiate a transfer by polling the busy busy bit from the IRQSTATUS_RAW         *
*   register so that we can send over the data. After we confirm that the bus is free, we can then check if           *
*   we can transmit data over the bus by polling the XRDY bit from the IRQSTATUS_RAW register. We wait until          *
*   we can transmit the data before sending over the data. The data counter will decrement as this is happening.      *
*   The data that we are transmitting to the PCA9685, sets up the device, sets the servo frequency, and               *
*	then controls the servo by using PWM on LED8 to turn the servo 0 degrees, +90 degrees and -90 degrees. There's a  *
*	timer delay (2 seconds and 1 second ) between each PWM control. After the timer, the User LEDs from GPIO1 are     *
*	turned on in sequence to each rotation and are then turned off.                                                   *
*                                                                                                                     *
**********************************************************************************************************************/
//defines

#define HWREG(x) (*((volatile unsigned int *) (x)))

//BBB addresses and offsets for setting up I2C

#define CNTRL_MODULE 0x44E10000         //control module for the pin muxing
#define I2C2_BASE_ADDRESS 0x4819C000    //module I2C2 from L4_PER Memory Map
#define CM_PER_ADDRESS 0x44E00000       //module to turn on the clock

#define I2C2_OFFSET 0x44                //offset to turn on the module
#define SDA 0x978                       //offset for the data line
#define SCL 0x97C                       //offset for the clock line

#define PSC 0xB0                        //prescaler to scale the clock from 48MHz to 12MHz

#define SCLL 0xB4                       //clock line low time
#define SCLH 0xB8                       //clock line high time

#define CON 0xA4                        //configuration register
#define SYSS 0x90                       //system status register

#define SA 0xAC                         //slave address register

#define CNT 0x98                        //data counter register

#define IRQSTATUS_RAW 0x24              //status raw register

#define DATA 0x9C                       //data access register


#define Busy_Bit 1 << 12
#define Start_And_Stop_Bits 0x3
//BBB addresses and offsets for using GPIO1 Pins and Timer2

#define GPIO1_BASE_ADDRESS 0x4804C000   //module GPIO1 from L4_PER Memory Map
#define CM_PER_GPIO1_CLKCTRL 0xAC 		//turn on clock for GPIO1 for using LED0, LED1, LED2
#define SETDATAOUT 0x194 				//used to light up the LEDS
#define OUTPUT_ENABLE 0x134 			//enable the LEDs through RMW by using this offset
#define CLEARDATAOUT 0x190 				//used to turn off the LEDs

#define LED0_SET_and_CLEAR 0x00200000 	//used to set and clear the LED0 (write a 1 to pin 21)
#define LED0_RMW_MASK 0xFFDFFFFF 		//used to RMW the output enable to enable the LED0 (write a 0 to pin 21)

#define LED1_SET_and_CLEAR 0x00400000 	//used for set and clear the LED1 (write a 1 to pin 22)
#define LED1_RMW_MASK 0xFFBFFFFF 		//used to RMW the output enable to enable LED1 (write a 0 to pin 22)

#define LED2_SET_and_CLEAR 0x00800000 	//used for set and clear the LED2 (write a 1 to pin 23)
#define LED2_RMW_MASK 0xFF7FFFFF 		//used to RMW the otput enable to enable LED2 (write a 0 to pin 23)

#define TIMER2_BASE_ADDRESS 0x48040000	//module for Timer2 from L4_PER Memory Map
#define CM_PER_TIMER2_CLKCTRL 0x80 		//turn on clock for Timer2 in order to set the Timer2 to 1s or 2s
#define PRCMCLKSEL_TIMER2 0x508 		//set clock frequency multiplexer for 32.760 KHz
#define IRQSTATUS_RAW_TIMER2 0x24 		//going to check the IRQSTATIS_RAW for timer2 (to make sure overflow happened)
#define TCLR 0x38						//timer control register (value will be set to a 1 to begin counting)
#define TCRR 0x3C 						//timer counter (will have 1s or 2s value after TLDR)

#define ONE_SECOND  0x7FFF8000			//value of 1 second that will be put in TLDR and TCRR
#define TWO_SECONDS 0xFFFF0000			//value of 2 seconds that will be put in TLDR and TCRR


//PCA9685 addresses

#define MODE1 0x00                      //address to enable bits for SLEEP, ALLCALL, RESTART, etc
#define PRE_SCALE_SERVO 0xFE            //prescale address for servo
#define MODE2 0x01                      //address to enable totem pole structure and non-inverted

#define LED8_ON_H 0x27 					//address for LED8 on high
#define LED8_ON_L 0x26 					//address for LED8 on low

#define LED8_OFF_H 0x29 				//address for LED8 off high
#define LED8_OFF_L 0x28					//address for LED8 off low

void INITIALIZE_CON ( ); 				//will be used for init the configuration register values
void DELAY_COUNTER ( ); 				//used in init of the PCA

void ENABLE_GPIO1 ( ); 					//turn on GPIO1 to enable it
void ENABLE_TIMER2 ( ); 				//turn on Timer2 to enable it

void ON_LED0 ( ); 						//used to turn on LED0 at pin 21

void ON_LED1 ( ); 						//used to turn on LED1 at pin 22

void ON_LED2 ( ); 						//used to turn on LED2 at pin 23

void DELAY_1_SECOND ( ); 				//turn on timer2 for 1 second
void DELAY_2_SECONDS ( ); 				//turn on timer2 for 2 seconds

int main ( void )
{

    //enable the SCL and SDA lines for using I2C2 using the control module's base address
    //and do pin muxing on pins 19 and 20 to enable those lines (mode 3).
    //the hex value represents the fast slew, receiver enabled, pullup disable, and 3 in lowest
    //bits for the module
    HWREG ( CNTRL_MODULE + SCL ) = 0x0000002B;
    HWREG ( CNTRL_MODULE + SDA ) = 0x0000002B;

    //turn on the clock to enable I2C2
    HWREG (CM_PER_ADDRESS + I2C2_OFFSET ) = 0x02;

    //turn on the clock to enable GPIO1
    ENABLE_GPIO1 ( );

    // Step 1 to 12 Mhz
    //program the prescaler to scale down the clock from 48MHz to 12 MhZ
    HWREG ( I2C2_BASE_ADDRESS + PSC ) = 0x3;

    // Step 2, for 400 kbs
    //program the I2C clock
    HWREG ( I2C2_BASE_ADDRESS + SCLL ) = 0x8;
    HWREG ( I2C2_BASE_ADDRESS + SCLH ) = 0xA;

 	//poll for transferring and transmitting data
	unsigned int PCA_ADDRESSES_AND_OFFSETS [] = { MODE1, 0x11, PRE_SCALE_SERVO, 0x79, MODE1, 0x81, MODE2, 0x04 };
    //instructions to turn the servo 0 degrees
    unsigned int PCA_0_DEGREES [ ] = { LED8_ON_H, 0x00, LED8_ON_L, 0x00,  LED8_OFF_H , 0x1, LED8_OFF_L, 0x32 };

    //instructions to turn the servo 90 degrees
    unsigned int PCA_90_DEGREES [ ] = { LED8_ON_H , 0x00, LED8_ON_L , 0x00, LED8_OFF_H , 0x1, LED8_OFF_L , 0x99 };

    //instructions to turn the servo -90 degrees
    unsigned int PCA_NEGATIVE_90_DEGREES [ ] = { LED8_ON_H , 0x00, LED8_ON_L , 0x00, LED8_OFF_H , 0x00, LED8_OFF_L , 0xCC }; 

    //repeat the below sequence of steps 5 times
    for (int i = 0; i < 5; i++){


  	/****************************
    **INITIALIZE THE PCA DEVICE *
    *****************************
    */ 

	for (unsigned int j = 0; j < sizeof (PCA_ADDRESSES_AND_OFFSETS )/ sizeof (unsigned int); j+=2 ){



		//software reset of BBB
		HWREG ( I2C2_BASE_ADDRESS + 0x10 ) = 0x00000002;

		//buffer of clear fifo and set threshold bit 
		HWREG ( I2C2_BASE_ADDRESS + 0x94) = 0x41;

		HWREG ( I2C2_BASE_ADDRESS + CON ) = 0x00008600;

		while( HWREG (I2C2_BASE_ADDRESS + SYSS ) != 1 );  // wait until the system status register's reset is  done
		
		//configure the I2C_SA and I2C_CNT registers 
		//I will use the slave address 1000 000 or 0x40 because
		//A5-A0 are grounded in the schematic and the MSB is a always a 1
		HWREG ( I2C2_BASE_ADDRESS + SA ) = 0x40; 
		//set the counter to transfer the desired number of bytes
		HWREG ( I2C2_BASE_ADDRESS + CNT ) = 0x2;

		//begin the transfer by polling the BB bit 12 from IRQSTATUS_RAW register
		//if the bit is not 0, then wait
		while ( ( HWREG ( I2C2_BASE_ADDRESS + IRQSTATUS_RAW ) & 1 << 12 ) != 0x0 );

		//set the start and stop bits in the configuration register to 1
		HWREG ( I2C2_BASE_ADDRESS + CON ) |= Start_And_Stop_Bits;

		for( int delay = 0; delay < 5000; delay++ ){
			asm ("NOP");
		}

		for (unsigned int i = 0; i < 2; i++ ){
			//wait until bit 4 (XRDY) is 1
			while ( ( HWREG ( I2C2_BASE_ADDRESS + IRQSTATUS_RAW ) & 1 << 4 ) == 0 );
			//transmit the commands
			HWREG ( I2C2_BASE_ADDRESS + DATA ) = PCA_ADDRESSES_AND_OFFSETS[j+i];

			for( int delay = 0; delay < 5000; delay++ )
			{
				asm ("NOP");
			}

			// Clears the XRDY
			HWREG ( I2C2_BASE_ADDRESS + IRQSTATUS_RAW ) |= 1<<4;
			
		}

		//check if the busy free bit has been set
		while ( ( HWREG ( I2C2_BASE_ADDRESS + IRQSTATUS_RAW ) & 1 << 8 ) !=  1 << 8 );
	}


    /*****************************
    ** TURN SERVO 0 DEGREES      *
    ******************************
    */

	for (unsigned int j = 0; j < sizeof ( PCA_0_DEGREES )/ sizeof (unsigned int); j+=2 ){

		//software reset of BBB
		HWREG ( I2C2_BASE_ADDRESS + 0x10 ) = 0x00000002;

		//buffer of clear fifo and set threshold bit 
		HWREG ( I2C2_BASE_ADDRESS + 0x94) = 0x41;

		HWREG ( I2C2_BASE_ADDRESS + CON ) = 0x00008600;

		while( HWREG (I2C2_BASE_ADDRESS + SYSS ) != 1 );  // wait until the system status register's reset is  done
		
		//configure the I2C_SA and I2C_CNT registers 
		//I will use the slave address 1000 000 or 0x40 because
		//A5-A0 are grounded in the schematic and the MSB is a always a 1
		HWREG ( I2C2_BASE_ADDRESS + SA ) = 0x40; 
		//set the counter to transfer the desired number of bytes
		HWREG ( I2C2_BASE_ADDRESS + CNT ) = 0x2;

		//begin the transfer by polling the BB bit 12 from IRQSTATUS_RAW register
		//if the bit is not 0, then wait
		while ( ( HWREG ( I2C2_BASE_ADDRESS + IRQSTATUS_RAW ) & 1 << 12 ) != 0x0 );

		//set the start and stop bits in the configuration register to 1
		HWREG ( I2C2_BASE_ADDRESS + CON ) |= Start_And_Stop_Bits;


		for( int delay = 0; delay < 5000; delay++ ){
			asm ("NOP");
		}

		for (unsigned int i = 0; i < 2; i++ ){
			//wait until bit 4 (XRDY) is 1
			while ( ( HWREG ( I2C2_BASE_ADDRESS + IRQSTATUS_RAW ) & 1 << 4 ) == 0 );
			//transmit the commands
			HWREG ( I2C2_BASE_ADDRESS + DATA ) = PCA_0_DEGREES[j+i];

			for( int delay = 0; delay < 5000; delay++ )
			{
				asm ("NOP");
			}

			// Clears the XRDY
			HWREG ( I2C2_BASE_ADDRESS + IRQSTATUS_RAW ) |= 1<<4;
			
		}

		//check if the busy free bit has been set
		while ( ( HWREG ( I2C2_BASE_ADDRESS + IRQSTATUS_RAW ) & 1 << 8 ) !=  1 << 8 );

		

    
	}

	/****************************
	** Delay for 2 seconds      *
	*****************************
	*/

	DELAY_2_SECONDS ( );

	/****************************
    ** TURN on LED0             *
    *****************************
    */

    ON_LED0 ( );


    /****************************
    ** TURN SERVO 90 DEGREES    *
    *****************************
    */

    for (unsigned int j = 0; j < sizeof ( PCA_90_DEGREES )/ sizeof (unsigned int); j+=2 ){

		//software reset of BBB
		HWREG ( I2C2_BASE_ADDRESS + 0x10 ) = 0x00000002;

		//buffer of clear fifo and set threshold bit 
		HWREG ( I2C2_BASE_ADDRESS + 0x94) = 0x41;

		HWREG ( I2C2_BASE_ADDRESS + CON ) = 0x00008600;

		while( HWREG (I2C2_BASE_ADDRESS + SYSS ) != 1 );  // wait until the system status register's reset is  done
		
		//configure the I2C_SA and I2C_CNT registers 
		//I will use the slave address 1000 000 or 0x40 because
		//A5-A0 are grounded in the schematic and the MSB is a always a 1
		HWREG ( I2C2_BASE_ADDRESS + SA ) = 0x40; 
		//set the counter to transfer the desired number of bytes
		HWREG ( I2C2_BASE_ADDRESS + CNT ) = 0x2;

		//begin the transfer by polling the BB bit 12 from IRQSTATUS_RAW register
		//if the bit is not 0, then wait
		while ( ( HWREG ( I2C2_BASE_ADDRESS + IRQSTATUS_RAW ) & 1 << 12 ) != 0x0 );

		//set the start and stop bits in the configuration register to 1
		HWREG ( I2C2_BASE_ADDRESS + CON ) |= Start_And_Stop_Bits;


		for( int delay = 0; delay < 5000; delay++ ){
			asm ("NOP");
		}

		for (unsigned int i = 0; i < 2; i++ ){
			//wait until bit 4 (XRDY) is 1
			while ( ( HWREG ( I2C2_BASE_ADDRESS + IRQSTATUS_RAW ) & 1 << 4 ) == 0 );
			//transmit the commands
			HWREG ( I2C2_BASE_ADDRESS + DATA ) = PCA_90_DEGREES[j+i];

			for( int delay = 0; delay < 5000; delay++ )
			{
				asm ("NOP");
			}

			// Clears the XRDY
			HWREG ( I2C2_BASE_ADDRESS + IRQSTATUS_RAW ) |= 1<<4;
			
		}

		//check if the busy free bit has been set
		while ( ( HWREG ( I2C2_BASE_ADDRESS + IRQSTATUS_RAW ) & 1 << 8 ) !=  1 << 8 );

	}
 
	/****************************
	** Delay for 1 second       *
	*****************************
	*/

    DELAY_1_SECOND ( );

	/****************************
    ** TURN on LED1             *
    *****************************
    */

    ON_LED1 ( );


    /*****************************
    ** TURN SERVO -90 DEGREES    *
    ******************************
    */

	for (unsigned int j = 0; j < sizeof ( PCA_NEGATIVE_90_DEGREES )/ sizeof (unsigned int); j+=2 ){

		//software reset of BBB
		HWREG ( I2C2_BASE_ADDRESS + 0x10 ) = 0x00000002;

		//buffer of clear fifo and set threshold bit 
		HWREG ( I2C2_BASE_ADDRESS + 0x94) = 0x41;

		HWREG ( I2C2_BASE_ADDRESS + CON ) = 0x00008600;

		while( HWREG (I2C2_BASE_ADDRESS + SYSS ) != 1 );  // wait until the system status register's reset is  done
		
		//configure the I2C_SA and I2C_CNT registers 
		//I will use the slave address 1000 000 or 0x40 because
		//A5-A0 are grounded in the schematic and the MSB is a always a 1
		HWREG ( I2C2_BASE_ADDRESS + SA ) = 0x40; 
		//set the counter to transfer the desired number of bytes
		HWREG ( I2C2_BASE_ADDRESS + CNT ) = 0x2;

		//begin the transfer by polling the BB bit 12 from IRQSTATUS_RAW register
		//if the bit is not 0, then wait
		while ( ( HWREG ( I2C2_BASE_ADDRESS + IRQSTATUS_RAW ) & 1 << 12 ) != 0x0 );

		//set the start and stop bits in the configuration register to 1
		HWREG ( I2C2_BASE_ADDRESS + CON ) |= Start_And_Stop_Bits;


		for( int delay = 0; delay < 5000; delay++ ){
			asm ("NOP");
		}

		for (unsigned int i = 0; i < 2; i++ ){
			//wait until bit 4 (XRDY) is 1
			while ( ( HWREG ( I2C2_BASE_ADDRESS + IRQSTATUS_RAW ) & 1 << 4 ) == 0 );
			//transmit the commands
			HWREG ( I2C2_BASE_ADDRESS + DATA ) = PCA_NEGATIVE_90_DEGREES[j+i];

			for( int delay = 0; delay < 5000; delay++ )
			{
				asm ("NOP");
			}

			// Clears the XRDY
			HWREG ( I2C2_BASE_ADDRESS + IRQSTATUS_RAW ) |= 1<<4;
			
		}

		//check if the busy free bit has been set
		while ( ( HWREG ( I2C2_BASE_ADDRESS + IRQSTATUS_RAW ) & 1 << 8 ) !=  1 << 8 );

	}

	/****************************
	** Delay for 2 seconds      *
	*****************************
	*/

    DELAY_2_SECONDS ( );    

 
	/****************************
    ** TURN on LED2             *
    *****************************
    */

    ON_LED2 ( );

    }



    return 0;
}


//enable Timer2
void ENABLE_TIMER2 ( ){

	HWREG ( CM_PER_ADDRESS + CM_PER_TIMER2_CLKCTRL ) = 0x02; //turn on clock for Timer2
	HWREG ( CM_PER_ADDRESS + PRCMCLKSEL_TIMER2 ) = 0x2; 	 //select the 32 KHz clock
}

//turn on timer2 for 1 second
void DELAY_1_SECOND ( ){
	//enable the timer2
	ENABLE_TIMER2 ( );

	HWREG ( TIMER2_BASE_ADDRESS + 0x10 ) = 0x1; 			//reset timer2
	HWREG ( TIMER2_BASE_ADDRESS + TCRR ) = ONE_SECOND; 		//put the value in the timer counter
	HWREG ( TIMER2_BASE_ADDRESS + TCLR ) = 0x1; 			//write a 1 to the timer control register at bit 0 to start the timer


}

//turn on timer2 for 2 seconds
void DELAY_2_SECONDS ( ){

	//enable the timer2
	ENABLE_TIMER2 ( );

	HWREG ( TIMER2_BASE_ADDRESS + 0x10 ) = 0x1; 		//reset timer2
	HWREG ( TIMER2_BASE_ADDRESS + TCRR ) = TWO_SECONDS; //put the value in the timer counter
	HWREG ( TIMER2_BASE_ADDRESS + TCLR ) = 0x1; 		//write a 1 to the timer control register at bit 0 to start the timer

}

//enable GPIO1
void ENABLE_GPIO1 ( ){

	HWREG ( CM_PER_ADDRESS + CM_PER_GPIO1_CLKCTRL ) = 0x02; 
}


//turn on LED0
void ON_LED0 ( ){

	HWREG ( GPIO1_BASE_ADDRESS + SETDATAOUT ) = LED0_SET_and_CLEAR; //write a 1 to set data out for LED0
	HWREG ( GPIO1_BASE_ADDRESS + OUTPUT_ENABLE ) &= LED0_RMW_MASK;	//to turn on do a RMW for LED0
}

//turn on LED1
void ON_LED1 ( ){

	HWREG ( GPIO1_BASE_ADDRESS + SETDATAOUT ) = LED1_SET_and_CLEAR; //write a 1 to set data out for LED1
	HWREG ( GPIO1_BASE_ADDRESS + OUTPUT_ENABLE ) &= LED1_RMW_MASK; 	//to turn on do a RMW for LED1
}


//turn on LED2
void ON_LED2 ( ){

	HWREG ( GPIO1_BASE_ADDRESS + SETDATAOUT ) = LED2_SET_and_CLEAR; //write a 1 to set data out for LED2
	HWREG ( GPIO1_BASE_ADDRESS + OUTPUT_ENABLE ) &= LED2_RMW_MASK; 	//to turn on do a RMW for LED2
}


