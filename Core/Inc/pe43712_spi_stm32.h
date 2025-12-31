extern uint8_t pullstart;

#ifndef ntonl2
#  define ntonl2(n) (((((unsigned long)(n) & 0xFF)) << 8) | \
                  ((((unsigned long)(n) & 0xFF00)) >> 8))
#endif

//choose another MCU series if you are not using STM32F4.

//if you predefined pin names, you could find them in main.h and use them below

#define PE43712_SEN_PIN GPIO_PIN_5
#define PE43712_SEN_PORT GPIOD

//choose a Hardware SPI port to use.
#define PE43712_SPI_PORT hspi2
extern SPI_HandleTypeDef PE43712_SPI_PORT;



//Define the pins tp connect
#define PE43712_SEN_Clr() HAL_GPIO_WritePin(PE43712_SEN_PORT, PE43712_SEN_PIN, GPIO_PIN_RESET)
#define PE43712_SEN_Set() HAL_GPIO_WritePin(PE43712_SEN_PORT, PE43712_SEN_PIN, GPIO_PIN_SET)


/*

static void PE43712_toggleSEN()
{
        PE43712_SEN_Set();
        HAL_Delay(10);
        PE43712_SEN_Clr();
}
*/





void PE43712_writeReg(unsigned char reg_num, unsigned int value) {
      	unsigned char buffer[3];
      	unsigned int * cmdstart = (unsigned int *) (&buffer[0]);
	// 0xF8 is ignoring area
	// address 
      	//*cmdstart = ntonl2(((reg_num & 0x07) << 8) | ( value & 0x7F )); // << 3 for device address
      	*cmdstart = (((reg_num & 0x07) << 8) | ( value & 0x7F )); // << 3 for device address

      	PE43712_SEN_Clr();
      	DWT_Delay_us(1);
      	HAL_SPI_Transmit(&PE43712_SPI_PORT, buffer, 2, 100);

      	DWT_Delay_us(1);
      	PE43712_SEN_Set();
      	//DWT_Delay_us(10);

}


