
//Name: Mary Nehmeh ID: 9933977 --Data Networking Assignment part 2 

#include "main.h"
#include "Time_Delays.h"
#include "Clk_Config.h"

#include "LCD_Display.h"

#include <stdio.h>
#include <string.h>

#include "Small_7.h"
#include "Arial_9.h"
#include "Arial_12.h"
#include "Arial_24.h"

#include "stm32f4xx_ll_crc.h"

//Temperature Sensor I2C Address
#define TEMPADR 0x90

//EEPROM I2C Address
#define EEPROMADR 0xA0

//for LCD display 
char outputString [18];

//Packet specs 
uint16_t temperature = 0; // to store the temperature value
uint16_t eeprom_temperature =0; 
uint16_t mac_dest_high = 0xaaaa; 
uint16_t mac_dest_mid = 0xaaaa; 
uint16_t mac_dest_low = 0xaaaa; 
uint16_t mac_src_high = 0xbbbb; 
uint16_t mac_src_mid = 0xbbbb; 
uint16_t mac_src_low = 0xbbbb; 
uint16_t length = 0x2E; 
uint16_t payload = 0; 
	 
//for fcs field 
uint32_t get_fcs = 0; 
uint32_t fcs = 0; 
uint32_t new_fcs = 0; 

//for up - down movement 
int state_counter = 0; 

//function prototypes 
uint32_t joystick_centre (void);
 
uint32_t joystick_left (void);

uint32_t joystick_right (void);

uint32_t joystick_up (void);

uint32_t joystick_down (void);

void joystick_config(void);

void eeprom_write(uint16_t mac_dest_high, uint16_t mac_dest_mid, uint16_t mac_dest_low, uint16_t mac_src_high, uint16_t mac_src_mid, uint16_t mac_src_low, uint16_t length, uint16_t temperature,uint32_t fcs);

uint16_t eeprom_read_temperature(void); 


uint32_t eeprom_read_fcs(void);

void i2c_configure();

uint16_t get_temperature(void);

int up_or_down (int state_counter);

uint32_t calculate_CRC (uint16_t temperature);

void aknowledge_polling(void);
	
int main(void){
	
	//Init
  SystemClock_Config();/* Configure the system clock to 84.0 MHz */
	SysTick_Config_MCE2(us);	
	
	// configure the LCD
	Configure_LCD_Pins ();
	Configure_SPI1 ();
	Activate_SPI1 ();
	Clear_Screen ();
	Initialise_LCD_Controller ();
	set_font ((unsigned char*) Arial_12);


	//configure the joystick 
	joystick_config();

	
	//Configure LED: GPIO Port B, pin 4
	LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType (GPIOB, LL_GPIO_PIN_4, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_4, LL_GPIO_PULL_NO);
	LL_GPIO_SetPinSpeed (GPIOB, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_HIGH);
	
	// configure SCL as Alternate function, Open Drain, Pull Up:
	LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_8_15 (GPIOB, LL_GPIO_PIN_8, LL_GPIO_AF_4);
	LL_GPIO_SetPinOutputType (GPIOB, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_OPENDRAIN);
	LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_8, LL_GPIO_PULL_UP);
	
  // configure SDA as: Alternate, Open Drain, Pull Up:
	LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_8_15 (GPIOB, LL_GPIO_PIN_9, LL_GPIO_AF_4);
	LL_GPIO_SetPinOutputType (GPIOB, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_OPENDRAIN);
	LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);
	
   //enable CRC clock 
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);

	//configure I2C
	i2c_configure();

	//print MAC dest to screen 
	Clear_Screen(); 
	sprintf (outputString, "MAC dest:"); 
	put_string (0,0,outputString);
	sprintf (outputString,"%x%x%x",mac_dest_high,mac_dest_mid,mac_dest_low); // print to the LCD
	put_string (0,20,outputString);
	
	while(1){
			 
		if (joystick_up()) {
			LL_mDelay(400000);
			state_counter = state_counter-1; //display the field above
			state_counter=up_or_down(state_counter); //ensure counter does not go below zero 
		}
	 else if (joystick_right()) {
			eeprom_write(mac_dest_high, mac_dest_mid,mac_dest_low,mac_src_high,mac_src_mid,mac_src_low,length,temperature,get_fcs); //write the temperature value to the EEPROM
		  LL_mDelay(500000); 
		  up_or_down(state_counter); //stay at the field it was before 
		}
	 else if (joystick_left()){
			eeprom_temperature = eeprom_read_temperature(); //read the temperature from the eeprom 
			fcs = eeprom_read_fcs(); 
			Clear_Screen(); 
			sprintf (outputString, "Retrieved"); // print message 
			put_string(0,0,outputString); 
			LL_mDelay(500000);
			up_or_down(state_counter); // stay at the field it was before 
	 }
	 else if (joystick_centre()) {
		  temperature = get_temperature(); //get the temperature from the temperature sensor and save it to memory 
			LL_mDelay(500000); 
			get_fcs = calculate_CRC(temperature); //calculate the fcs field 
			up_or_down(state_counter); // stay at the field it was before 
	 }
	 else if (joystick_down()){
		 LL_mDelay(400000);
		 state_counter = state_counter+1; //display the field below 
		 state_counter=up_or_down(state_counter);	 //ensure that counter does not go above the FCS field 
	 }
	
}
} //end of main 

void i2c_configure(void) {
	//enable clocks and configure pins for I2C 
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_8, LL_GPIO_AF_4);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_OPENDRAIN);
	
	
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_9, LL_GPIO_AF_4);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_OPENDRAIN);
	
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
	
	LL_I2C_Disable(I2C1);
	LL_I2C_SetMode(I2C1, LL_I2C_MODE_I2C);
	LL_I2C_ConfigSpeed(I2C1, 84000000, 400000, LL_I2C_DUTYCYCLE_2);
	LL_I2C_Enable(I2C1);
}

uint32_t joystick_right (void) {
// returns 1 if the joystick is pressed in the right, or 0 otherwise
return (LL_GPIO_IsInputPinSet (GPIOC, LL_GPIO_PIN_0));
}

uint32_t joystick_left (void) {
// returns 1 if the joystick is pressed in the left, or 0 otherwise
return (LL_GPIO_IsInputPinSet (GPIOC, LL_GPIO_PIN_1));
}

uint32_t joystick_centre (void) {
// returns 1 if the joystick is pressed in the centre, or 0 otherwise
return (LL_GPIO_IsInputPinSet (GPIOB, LL_GPIO_PIN_5));
}

uint32_t joystick_up (void) {
// returns 1 if the joystick is pressed up, or 0 otherwise
return (LL_GPIO_IsInputPinSet (GPIOA, LL_GPIO_PIN_4));
}

uint32_t joystick_down (void) {
// returns 1 if the joystick is pressed down, or 0 otherwise
return (LL_GPIO_IsInputPinSet (GPIOB, LL_GPIO_PIN_0));
}


void eeprom_write(uint16_t mac_dest_high, uint16_t mac_dest_mid, uint16_t mac_dest_low, uint16_t mac_src_high, uint16_t mac_src_mid, uint16_t mac_src_low, uint16_t length, uint16_t temperature,uint32_t fcs){
		  // variables to split 16 bits into 8 for writing  
			uint16_t dest_h_h, dest_h_l, dest_m_h, dest_m_l, dest_l_h, dest_l_l; 
			uint16_t src_h_h, src_h_l, src_m_h, src_m_l, src_l_h, src_l_l; 
			uint16_t length_h, length_l;
			uint8_t payload = 0x00; 
			uint16_t temp_h, temp_l; 
			uint8_t fcs_h_h, fcs_h_l,fcs_l_h,fcs_l_l;
			
			//formatting MAC destination into 8 bit words
			dest_h_h = mac_dest_high >> 8;
		  dest_h_l  = mac_dest_high & 0x00FF;
			dest_m_h = mac_dest_mid >> 8;
		  dest_m_l  = mac_dest_mid & 0x00FF;
			dest_l_h = mac_dest_low >> 8;
		  dest_l_l  = mac_dest_low & 0x00FF;
			
			//formatting MAC source into 8 bit words
			src_h_h = mac_src_high >> 8;
		  src_h_l  = mac_src_high & 0x00FF;
			src_m_h = mac_dest_mid >> 8;
		  src_m_l  = mac_dest_mid & 0x00FF;
			src_l_h = mac_src_low >> 8;
		  src_l_l  = mac_src_low & 0x00FF;
			
			//formatting length into 8 bit words
			length_h = length >> 8;
		  length_l  = length & 0x00FF;
			
			
			//formatting temperature into 8 bit words
			temp_h = temperature >> 8;
		  temp_l  = temperature & 0x00FF;
			
			//formatting the fcs into 8 bit words 
			fcs_h_h = fcs >> 24; 
			fcs_h_l = fcs >> 16; 
			fcs_l_h = fcs >> 8;
			fcs_l_l = fcs;
			
		  LL_I2C_GenerateStartCondition (I2C1); // Start
			while (!LL_I2C_IsActiveFlag_SB (I2C1)); // wait until complete
			
	
			LL_I2C_TransmitData8 (I2C1, EEPROMADR); // Address + Write
			while (!LL_I2C_IsActiveFlag_ADDR (I2C1)); // wait until complete
			LL_I2C_ClearFlag_ADDR (I2C1); // clear the ADDR flag
			
	
			LL_I2C_TransmitData8 (I2C1, 0x00); // address HIGH byte
			while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
	    LL_I2C_TransmitData8 (I2C1, 0x00); // address LOW byte
			while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
		  
			//write MAC destination 
			LL_I2C_TransmitData8 (I2C1, dest_h_h); // Address + write
			while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
			LL_I2C_TransmitData8 (I2C1, dest_h_l); // Address + write
			while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
			LL_I2C_TransmitData8 (I2C1, dest_m_h); // Address + write
			while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
			LL_I2C_TransmitData8 (I2C1, dest_m_l); // Address + write
			while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
			LL_I2C_TransmitData8 (I2C1, dest_l_h); // Address + write
			while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
			LL_I2C_TransmitData8 (I2C1, dest_l_l); // Address + write
			while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
			
			//write MAC source 
			LL_I2C_TransmitData8 (I2C1, src_h_h); // Address + write
			while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
			LL_I2C_TransmitData8 (I2C1, src_h_l); // Address + write
			while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
			LL_I2C_TransmitData8 (I2C1, src_m_h); // Address + write
			while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
			LL_I2C_TransmitData8 (I2C1, src_m_l); // Address + write
			while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
			LL_I2C_TransmitData8 (I2C1, src_l_h); // Address + write
			while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
			LL_I2C_TransmitData8 (I2C1, src_l_l); // Address + write
			while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
			
			//write length
			LL_I2C_TransmitData8 (I2C1, length_h); // Address + write
			while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete 
			LL_I2C_TransmitData8 (I2C1, length_l); // Address + write
			while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete 
			
			//write payload empty fields 
			for (int i=0;i<18;i++) {
				LL_I2C_TransmitData8 (I2C1, payload); // Address + write
				while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete 
			}
			
			LL_I2C_GenerateStopCondition(I2C1); //generate stop condition 
		
			aknowledge_polling(); //allows for completion of the EEPROM internal write operation 

		  LL_I2C_ClearFlag_ADDR (I2C1); // clear the ADDR flag
			
			LL_I2C_TransmitData8 (I2C1, 0x00); // address HIGH byte
			while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
			
	    LL_I2C_TransmitData8 (I2C1, 0x00); // address LOW byte
			while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
			
			//write payload empty fields 
			for (int i=0;i<26;i++) {
				LL_I2C_TransmitData8 (I2C1, payload); // Address + write
				while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete 
			}
		
			//write temperature 
			LL_I2C_TransmitData8 (I2C1, temp_h); // Address + write
			while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
			LL_I2C_TransmitData8 (I2C1, temp_l); // Address + write
			while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
			
			//transmit FCS
			LL_I2C_TransmitData8 (I2C1, fcs_h_h); // Address + write
			while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
			LL_I2C_TransmitData8 (I2C1, fcs_h_l); // Address + write
			while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
			LL_I2C_TransmitData8 (I2C1, fcs_l_h); // Address + write
			while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
			LL_I2C_TransmitData8 (I2C1, fcs_l_l); // Address + write
			while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete

			LL_I2C_GenerateStopCondition(I2C1); //generate stop condition 
			
			LL_mDelay(5000);
			
			Clear_Screen(); 
			sprintf (outputString, "Written"); // print message to the LCD 
			put_string (0,0,outputString);
}

uint16_t get_temperature (void) {
				uint16_t temperature = 0; 
				LL_I2C_GenerateStartCondition (I2C1); // Start
				while (!LL_I2C_IsActiveFlag_SB (I2C1)); // wait until complete
				
				LL_I2C_TransmitData8 (I2C1, TEMPADR); // Address + Write
				while (!LL_I2C_IsActiveFlag_ADDR (I2C1)); // wait until complete
				LL_I2C_ClearFlag_ADDR (I2C1); // clear the ADDR flag
				
				LL_I2C_TransmitData8 (I2C1, 0x00); // set pointer register to the temperature register
				while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete

				LL_I2C_GenerateStartCondition (I2C1); // Restart (when a read command follows a write)
				while (!LL_I2C_IsActiveFlag_SB (I2C1)); // wait until complete
				
				LL_I2C_TransmitData8 (I2C1, TEMPADR+1); // Address + Read
				while (!LL_I2C_IsActiveFlag_ADDR (I2C1)); // wait until complete
				LL_I2C_ClearFlag_ADDR (I2C1); // clear the ADDR flag
				
	
				LL_I2C_AcknowledgeNextData (I2C1, LL_I2C_ACK); // ACK incoming data
				while (!LL_I2C_IsActiveFlag_RXNE (I2C1)); // wait until complete
				
				temperature = LL_I2C_ReceiveData8 (I2C1); // read temperature High Byte from Receive Data register.
				
				temperature = temperature << 8; //format to MSB 

				LL_I2C_AcknowledgeNextData (I2C1, LL_I2C_NACK); // NACK incoming data
				while (!LL_I2C_IsActiveFlag_RXNE (I2C1)); // wait until complete
				temperature += LL_I2C_ReceiveData8 (I2C1); // read temperature Low Byte

				LL_I2C_GenerateStopCondition (I2C1); // Stop
				temperature = temperature >> 5; // bit-shift temperature right, since it's stored in the upper 11 bits of the two bytes.	
				
				Clear_Screen(); 
				sprintf (outputString, "Sampled"); // print message to the LCD
				put_string (0,0,outputString);
				LL_mDelay(500000); 
				Clear_Screen(); 
				sprintf (outputString, "Temp(*C):"); 
				put_string (0,0,outputString);
				sprintf (outputString, "%f", temperature*0.125); // print sampled temperature the LCD
				put_string (0,20,outputString);
				return temperature; 
		} 

uint16_t eeprom_read_temperature(void) {
	
				uint16_t temperature; 
				uint16_t dummy; // for unwanted received data 
				LL_I2C_GenerateStartCondition (I2C1); // Start
				while (!LL_I2C_IsActiveFlag_SB (I2C1)); // wait until complete
				
				LL_I2C_TransmitData8 (I2C1,EEPROMADR); // Address + Write
				while (!LL_I2C_IsActiveFlag_ADDR (I2C1)); // wait until complete
				LL_I2C_ClearFlag_ADDR (I2C1); // clear the ADDR flag
				
				LL_I2C_TransmitData8 (I2C1, 0x00); // address HIGH byte
				while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
				LL_I2C_TransmitData8 (I2C1, 0x00); // address LOW byte
				while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
		
				LL_I2C_GenerateStartCondition (I2C1); // Restart (when a read command follows a write)
				while (!LL_I2C_IsActiveFlag_SB (I2C1)); // wait until complete
				
				LL_I2C_TransmitData8 (I2C1, EEPROMADR + 1); // Address + Read
				while (!LL_I2C_IsActiveFlag_ADDR (I2C1)); // wait until complete
				LL_I2C_ClearFlag_ADDR (I2C1); // clear the ADDR flag
		
			
				LL_I2C_AcknowledgeNextData (I2C1, LL_I2C_ACK); // ACK incoming data
				for (int i=0;i<32;i++) {
				while (!LL_I2C_IsActiveFlag_RXNE (I2C1)); // wait until complete
				dummy = LL_I2C_ReceiveData8 (I2C1); // dump unwanted data
					}
	
				LL_I2C_GenerateStopCondition (I2C1); //restart 
				LL_mDelay(5000);
					
				LL_I2C_GenerateStartCondition (I2C1); // Start
				while (!LL_I2C_IsActiveFlag_SB (I2C1)); // wait until complete
				
				LL_I2C_TransmitData8 (I2C1,EEPROMADR); // Address + Write
				while (!LL_I2C_IsActiveFlag_ADDR (I2C1)); // wait until complete
				LL_I2C_ClearFlag_ADDR (I2C1); // clear the ADDR flag
				

				LL_I2C_TransmitData8 (I2C1, 0x00); // address HIGH byte
				while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
				LL_I2C_TransmitData8 (I2C1, 0x00); // address LOW byte
				while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
				
				
				LL_I2C_GenerateStartCondition (I2C1); // Restart (when a read command follows a write)
				while (!LL_I2C_IsActiveFlag_SB (I2C1)); // wait until complete
				
				LL_I2C_TransmitData8 (I2C1, EEPROMADR + 1); // Address + Read
				while (!LL_I2C_IsActiveFlag_ADDR (I2C1)); // wait until complete
				LL_I2C_ClearFlag_ADDR (I2C1); // clear the ADDR flag
				
				LL_I2C_AcknowledgeNextData (I2C1, LL_I2C_ACK); // ACK incoming data
				for (int i=0;i<26;i++) {
					while (!LL_I2C_IsActiveFlag_RXNE (I2C1)); // wait until complete
					dummy = LL_I2C_ReceiveData8 (I2C1); // dump unwanted data
					} 
		
				LL_I2C_AcknowledgeNextData (I2C1, LL_I2C_ACK); // ACK incoming data
				while (!LL_I2C_IsActiveFlag_RXNE (I2C1)); // wait until complete
				temperature = LL_I2C_ReceiveData8 (I2C1); // read temperature High Byte from Receive Data register.
				temperature = temperature << 8; //move to MSB
				
				LL_I2C_AcknowledgeNextData (I2C1, LL_I2C_ACK); // ACK incoming data
				while (!LL_I2C_IsActiveFlag_RXNE (I2C1)); // wait until complete
				temperature += LL_I2C_ReceiveData8 (I2C1); // read temperature Low Byte
				
				LL_I2C_AcknowledgeNextData (I2C1, LL_I2C_ACK); // ACK incoming data
					
				for (int i=0; i<3; i++) {
						while (!LL_I2C_IsActiveFlag_RXNE (I2C1)); // wait until complete
						dummy = LL_I2C_ReceiveData8 (I2C1); // dump unwanted data
				}
				
				LL_I2C_AcknowledgeNextData (I2C1, LL_I2C_NACK); // NACK incoming data
				while (!LL_I2C_IsActiveFlag_RXNE (I2C1)); // wait until complete
				dummy = LL_I2C_ReceiveData8 (I2C1);
				
				LL_I2C_GenerateStopCondition (I2C1); // Stop
					
				return temperature; 
		}

uint32_t eeprom_read_fcs(void) {
	
				uint32_t fcs; 
				uint16_t dummy; // for unwanted received data 
				LL_I2C_GenerateStartCondition (I2C1); // Start
				while (!LL_I2C_IsActiveFlag_SB (I2C1)); // wait until complete
				
				LL_I2C_TransmitData8 (I2C1,EEPROMADR); // Address + Write
				while (!LL_I2C_IsActiveFlag_ADDR (I2C1)); // wait until complete
				LL_I2C_ClearFlag_ADDR (I2C1); // clear the ADDR flag
				
				LL_I2C_TransmitData8 (I2C1, 0x00); // address HIGH byte
				while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
				LL_I2C_TransmitData8 (I2C1, 0x00); // address LOW byte
				while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
		
				LL_I2C_GenerateStartCondition (I2C1); // Restart (when a read command follows a write)
				while (!LL_I2C_IsActiveFlag_SB (I2C1)); // wait until complete
				
				LL_I2C_TransmitData8 (I2C1, EEPROMADR + 1); // Address + Read
				while (!LL_I2C_IsActiveFlag_ADDR (I2C1)); // wait until complete
				LL_I2C_ClearFlag_ADDR (I2C1); // clear the ADDR flag
		
			
				LL_I2C_AcknowledgeNextData (I2C1, LL_I2C_ACK); // ACK incoming data
				for (int i=0;i<32;i++) {
				while (!LL_I2C_IsActiveFlag_RXNE (I2C1)); // wait until complete
				dummy = LL_I2C_ReceiveData8 (I2C1); // dump unwanted data
					}
	
				LL_I2C_GenerateStopCondition (I2C1); //restart 
				LL_mDelay(5000);
					
				LL_I2C_GenerateStartCondition (I2C1); // Start
				while (!LL_I2C_IsActiveFlag_SB (I2C1)); // wait until complete
				
				LL_I2C_TransmitData8 (I2C1,EEPROMADR); // Address + Write
				while (!LL_I2C_IsActiveFlag_ADDR (I2C1)); // wait until complete
				LL_I2C_ClearFlag_ADDR (I2C1); // clear the ADDR flag
				

				LL_I2C_TransmitData8 (I2C1, 0x00); // address HIGH byte
				while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
				LL_I2C_TransmitData8 (I2C1, 0x00); // address LOW byte
				while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
				
				
				LL_I2C_GenerateStartCondition (I2C1); // Restart (when a read command follows a write)
				while (!LL_I2C_IsActiveFlag_SB (I2C1)); // wait until complete
				
				LL_I2C_TransmitData8 (I2C1, EEPROMADR + 1); // Address + Read
				while (!LL_I2C_IsActiveFlag_ADDR (I2C1)); // wait until complete
				LL_I2C_ClearFlag_ADDR (I2C1); // clear the ADDR flag
				
				LL_I2C_AcknowledgeNextData (I2C1, LL_I2C_ACK); // ACK incoming data
				for (int i=0;i<28;i++) {
					while (!LL_I2C_IsActiveFlag_RXNE (I2C1)); // wait until complete
					dummy = LL_I2C_ReceiveData8 (I2C1); // dump unwanted data
					} 
		
				LL_I2C_AcknowledgeNextData (I2C1, LL_I2C_ACK); // ACK incoming data
				while (!LL_I2C_IsActiveFlag_RXNE (I2C1)); // wait until complete
				fcs = LL_I2C_ReceiveData8 (I2C1); // read temperature High Byte from Receive Data register.
				fcs = fcs << 24; //move to MSB
				
				LL_I2C_AcknowledgeNextData (I2C1, LL_I2C_ACK); // ACK incoming data
				while (!LL_I2C_IsActiveFlag_RXNE (I2C1)); // wait until complete
				fcs += (LL_I2C_ReceiveData8 (I2C1))<<16; // read temperature Low Byte
				
				LL_I2C_AcknowledgeNextData (I2C1, LL_I2C_ACK); // ACK incoming data
				while (!LL_I2C_IsActiveFlag_RXNE (I2C1)); // wait until complete
				fcs += (LL_I2C_ReceiveData8 (I2C1))<<8; // dump unwanted data
				
				
				LL_I2C_AcknowledgeNextData (I2C1, LL_I2C_NACK); // NACK incoming data
				while (!LL_I2C_IsActiveFlag_RXNE (I2C1)); // wait until complete
				fcs += LL_I2C_ReceiveData8 (I2C1);
				
				LL_I2C_GenerateStopCondition (I2C1); // Stop
					
				return fcs; 
		}
int up_or_down (int state_counter) { //MAC destination 
			 if (state_counter == 0 || state_counter < 0) { //for MAC destination 
				Clear_Screen(); 
				sprintf (outputString, "MAC dest:"); // print to the LCD
				put_string (0,0,outputString);
				sprintf (outputString,"%x%x%x",mac_dest_high,mac_dest_mid,mac_dest_low); // print MAC destination to the LCD
				put_string (0,20,outputString);
				state_counter = 0;  //ensure counter does not go below zero 
		 }
		 else if (state_counter == 1) { //MAC source 
			 Clear_Screen(); 
			 sprintf (outputString, "MAC src:"); // print to the LCD
			 put_string (0,0,outputString);
			 sprintf (outputString,"%x%x%x",mac_src_high,mac_src_mid,mac_src_low); // print MAC source the LCD
			 put_string (0,20,outputString);
		 }
		 else if (state_counter == 2) { //length
			 Clear_Screen(); 
			 sprintf (outputString, "Length:"); // print to the LCD
			 put_string (0,0,outputString);
			 sprintf (outputString,"%x",length); // print length to the LCD
			 put_string (0,20,outputString);
		 }
		 else if (state_counter == 3) { //temperature
			 Clear_Screen(); 
			 sprintf (outputString, "Temp(*C):"); // print to the LCD
			 put_string (0,0,outputString);
			 sprintf (outputString,"%f",eeprom_temperature*0.125); // print temperature to the LCD
			 put_string (0,20,outputString);
			 state_counter = 3; 
		 }
		 else if (state_counter == 4 ) { //FCS
			 Clear_Screen(); 
			 sprintf (outputString, "FCS:"); // print to the LCD
			 put_string (0,0,outputString);
			 sprintf (outputString,"%x",fcs); // print FCS to the LCD
			 put_string (0,20,outputString);
			 state_counter = 4;
		 }
		else if( state_counter == 5 || state_counter > 5) { //FCS ok 
				Clear_Screen(); 
				new_fcs = calculate_CRC(temperature); //recalculate fcs 
				if (new_fcs == fcs) { //check if new fcs is the same as the old fcs 
					Clear_Screen(); 
					sprintf (outputString, "FCS CHECK OK"); // print to the LCD
					put_string (0,0,outputString); 
					sprintf (outputString, "%x",new_fcs); // print to the LCD
					put_string (0,20,outputString); 
					state_counter = 5; 
				}
					
				else{ // if new fcs not equal to old fcs then print an error 
					Clear_Screen(); 
					sprintf (outputString, "FCS ERROR"); // print to the LCD
					put_string (0,0,outputString); 
				}
				sprintf (outputString, "FCS:"); // print to the LCD
				put_string (0,0,outputString); 
				sprintf (outputString, "%x",new_fcs); // print to the LCD
				put_string (0,20,outputString); 
		 }
		 return state_counter; 
	 }

 uint32_t calculate_CRC(uint16_t temperature) {
		LL_CRC_ResetCRCCalculationUnit(CRC);
	 
		uint32_t CRC_input,CRC_output; 	
	
		CRC_input = (mac_dest_high<<8)|mac_dest_mid; //feed 4 bytes to CRC 
		LL_CRC_FeedData32(CRC,CRC_input);  
		LL_mDelay(50); 
		
		CRC_input = (mac_dest_low<<8)|mac_src_high; //feed 4 bytes to CRC
		LL_CRC_FeedData32(CRC,CRC_input);
		LL_mDelay(50);
	  
		CRC_input = (mac_src_mid<<8)|mac_src_low; //feed 4 bytes to CRC
		LL_CRC_FeedData32(CRC,CRC_input);
		LL_mDelay(50);
	 
		CRC_input = (length<<8)|payload; //feed 4 bytes to CRC
		LL_CRC_FeedData32(CRC,CRC_input);
		LL_mDelay(50);
	  
		for (int i=0;i<10;i++) { //feed 4 bytes to CRC (payload)
			CRC_input = (payload<<8)|payload;
			LL_CRC_FeedData32(CRC,CRC_input);
			LL_mDelay(50);
	  }
		
		CRC_input = (payload<<8)|temperature; //feed 4 bytes to CRC
		LL_CRC_FeedData32(CRC,CRC_input);
		LL_mDelay(50);
		
		CRC_output = LL_CRC_ReadData32(CRC); //read the CRC calculation 
		return CRC_output; 
	}
 
void aknowledge_polling(void) {
  LL_I2C_ClearFlag_AF(I2C1);
	LL_I2C_GenerateStartCondition (I2C1); //issue a start 
	while (!LL_I2C_IsActiveFlag_SB (I2C1)); //wait for the SB flag
	   
	LL_I2C_TransmitData8 (I2C1, EEPROMADR); // Address + Write
	LL_mDelay(100);
	
	while(LL_I2C_IsActiveFlag_AF(I2C1)==1) { //if AF flag is set repeat the loop - internal write operation not finished  
		LL_I2C_ClearFlag_AF(I2C1);
		
		LL_I2C_GenerateStartCondition (I2C1); //issue a start 
		while (!LL_I2C_IsActiveFlag_SB (I2C1)); //wait for the SB flag 
	   
		LL_I2C_TransmitData8 (I2C1, EEPROMADR); // Address + Write
		LL_mDelay(100);
		 }
	}

	void joystick_config(void) { //configure clocks and pins for joystick 
		
	LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOB); // enable peripheral clock 
	LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOC); // enable peripheral clock 
	LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOA); // enable peripheral clock 
  LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_INPUT); // set B5 as Input (centre)
  LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_5, LL_GPIO_PULL_NO); //set as NO pull
	
	LL_GPIO_SetPinMode (GPIOC, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT); // set C0 as Input (right)
  LL_GPIO_SetPinPull (GPIOC, LL_GPIO_PIN_0, LL_GPIO_PULL_NO); //set as NO pull
	
	LL_GPIO_SetPinMode (GPIOC, LL_GPIO_PIN_1, LL_GPIO_MODE_INPUT); // set C1 as Input (left)
  LL_GPIO_SetPinPull (GPIOC, LL_GPIO_PIN_1, LL_GPIO_PULL_NO); //set as NO pull
	
	LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT); // set C1 as Input (left)
  LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_0, LL_GPIO_PULL_NO); //set as NO pull
	
	LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_INPUT); // set C1 as Input (left)
  LL_GPIO_SetPinPull (GPIOA, LL_GPIO_PIN_4, LL_GPIO_PULL_NO); //set as NO pull
	}
