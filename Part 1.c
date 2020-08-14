
/*********************************************************************************************************************
*   Rebeka Henry                                                                                                     *
*   Programming I2C                                                                                                  *
*   7/31/2020                                                                                                        *
*   The program below implements I2C using the Beaglebone Black and the PCA9685 Servo Controller.                    *
*   In order to set up I2C, the control module, I2C2, and CM_PER addresses are used to initialize                    *
*   the BBB as the Master Transmitting Device. This essentially entails turning on the clock, modules,               *
*   pin muxing, prescaling the speed of the clock, etc.The sequence of steps follow the How to program               *
*   I2C from the Sitara Manual. In order to transmit data to control the PCA9685, we use the configuration           *
*   register to enable I2C2 and make the BB the Master Transmitter. Before transmitting the data, we also            *
*   wait for the system status register on the bus to give us the signal to set the slave address and the            *   
*   data counter. The next step is to initiate a transfer by polling the busy busy bit from the IRQSTATUS_RAW        *
*   register so that we can send over the data. After we confirm that the bus is free, we can then check if          *
*   we can transmit data over the bus by polling the XRDY bit from the IRQSTATUS_RAW register. We wait until         *
*   we can transmit the data before sending over the data. The data counter will decrement as this is happening.     *
*   The data that we are transmitting to the PCA9685, sets up the device, sets the servo frequency, and turns on     *
*   the LED15 that is located on the PCA device.                                                                     *
*                                                                                                                    *
**********************************************************************************************************************/

//defines

#define HWREG(x) (*((volatile unsigned int *) (x)))

//BBB addresses and offsets

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

//PCA9685 addresses

#define MODE1 0x00                      //address to enable bits for SLEEP, ALLCALL, RESTART, etc

#define PRE_SCALE_SERVO 0xFE            //prescale address for servo

#define MODE2 0x01                      //address to enable totem pole structure and non-inverted

#define LED15_ON_H 0x43                 //address for LED15 FULL_ON (going to be enabled using a 1)
#define LED15_OFF_H 0x45                //address for LED15 FULL_OFF (going to be disabled by writing a 0)

#define Busy_Bit 1 << 12
#define Start_And_Stop_Bits 0x3

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

	// Step 1 to 12 Mhz
	//program the prescaler to scale down the clock from 48MHz to 12 MhZ
	HWREG ( I2C2_BASE_ADDRESS + PSC ) = 0x3;

	// Step 2, for 400 kbs
	//program the I2C clock
	HWREG ( I2C2_BASE_ADDRESS + SCLL ) = 0x8;
	HWREG ( I2C2_BASE_ADDRESS + SCLH ) = 0xA;

	//poll for transferring and transmitting data
	unsigned int PCA_ADDRESSES_AND_OFFSETS [] = { MODE1, 0x11, PRE_SCALE_SERVO, 0x79, MODE1, 0x81, MODE2, 0x04, LED15_OFF_H, 0x00, LED15_ON_H, 0x10 };

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

			for( int delay = 0; delay < 5000; delay++ ){
				asm ("NOP");
			}

			// Clears the XRDY
			HWREG ( I2C2_BASE_ADDRESS + IRQSTATUS_RAW ) |= 1<<4;
			
		}

		//check if the busy free bit has been set and reset it if it has
		while ( ( HWREG ( I2C2_BASE_ADDRESS + IRQSTATUS_RAW ) & 1 << 8 ) !=  1 << 8 );
	}
	
	return 0;
}
