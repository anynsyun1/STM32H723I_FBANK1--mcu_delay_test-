extern uint8_t pullstart;

#ifndef htonl
#  define htonl(n) (((((unsigned long)(n) & 0xFF)) << 24) | \
                  ((((unsigned long)(n) & 0xFF00)) << 8) | \
                  ((((unsigned long)(n) & 0xFF0000)) >> 8) | \
                  ((((unsigned long)(n) & 0xFF000000)) >> 24))
#endif

#ifndef ntohl
#  define ntohl(n) (((((unsigned long)(n) & 0xFF)) << 24) | \
                  ((((unsigned long)(n) & 0xFF00)) << 8) | \
                  ((((unsigned long)(n) & 0xFF0000)) >> 8) | \
                  ((((unsigned long)(n) & 0xFF000000)) >> 24))
#endif


//choose another MCU series if you are not using STM32F4.

//if you predefined pin names, you could find them in main.h and use them below

#define HMC832_SEN_PIN GPIO_PIN_3
#define HMC832_SEN_PORT GPIOA

//choose a Hardware SPI port to use.
#define HMC832_SPI_PORT hspi1
extern SPI_HandleTypeDef HMC832_SPI_PORT;



//Define the pins tp connect
#define HMC832_SEN_Clr() HAL_GPIO_WritePin(HMC832_SEN_PORT, HMC832_SEN_PIN, GPIO_PIN_RESET)
#define HMC832_SEN_Set() HAL_GPIO_WritePin(HMC832_SEN_PORT, HMC832_SEN_PIN, GPIO_PIN_SET)




void SPI_WrRd(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	/*
	if(NumByteToRead > 0x01){
		ReadAddr |= (uint8_t)(0x80 | 0x40);
	}
	else{
		ReadAddr |= (uint8_t)0x80;
	}
	*/

	// set chip select low
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
	/// send the address of the indexed register

	HAL_SPI_Transmit(&HMC832_SPI_PORT, &ReadAddr, 1, 100);
	// receive the data from slave device (MSB first)
	NumByteToRead = 0x03;
	while(NumByteToRead > 0x00){
		// send dummy byte (0x00) to generate SPI clock to GYROSCOPE slave device
		HAL_SPI_Receive(&HMC832_SPI_PORT, pBuffer, 1, 100);
		NumByteToRead--;
		pBuffer++;
	}
	// set chip select high //
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
}


void SPI_Wr(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
	// configure the MS bit:
	// - when 0, address will remain unchanged in multiple read/write commands
	// - when 1, address will auto increment in multiple read/write commands

	/*
	if(NumByteToWrite > 0x01){
		WriteAddr |= 0x40;
	}
	*/
	// set chip select low
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
	// send address of the indexed register
	HAL_SPI_Transmit(&HMC832_SPI_PORT, &WriteAddr, 1, 100);
	// send the data to slave device (MSB first)
	while(NumByteToWrite >= 0x01){
		HAL_SPI_Transmit(&HMC832_SPI_PORT, pBuffer, 1, 100);
		NumByteToWrite--;
		pBuffer++;
	}
	// set chip select high
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
}


unsigned int HMC832_readRegmy(unsigned char reg_num, unsigned int value) {
	unsigned char buffer[5];
	unsigned int * cmdstart = (unsigned int *) (&buffer[0]);
  	*cmdstart = ntohl((value << 8) | ((reg_num & 0x1F) << 3)); // << 3 for device address

  	HMC832_SEN_Clr();
  	DWT_Delay_us(10);
  	HAL_SPI_Transmit(&HMC832_SPI_PORT, buffer, 4, 100);

  	DWT_Delay_us(10);
  	HMC832_SEN_Set();
  	DWT_Delay_us(10);
  	HMC832_SEN_Clr();
  	DWT_Delay_us(10);
	//HAL_SPI_Transmit(&HMC832_SPI_PORT, buffer, 4, 100);
      	uint16_t NumByteToRead = 0x04;
    	uint8_t *pBuffer=buffer;
    	while(NumByteToRead > 0x00){
    		// send dummy byte (0x00) to generate SPI clock to GYROSCOPE slave device
    		HAL_SPI_Receive(&HMC832_SPI_PORT, pBuffer, 1, 100);
    		NumByteToRead--;
    		pBuffer++;
    	}

    	__NOP();
      	HAL_SPI_Init(&HMC832_SPI_PORT);
      	DWT_Delay_us(10);
      	HMC832_SEN_Set();

      	unsigned int readResultShifted = (buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + (buffer[3] << 0);
      	unsigned int readRegValue = (readResultShifted >> 8);
      	return readRegValue;
}


unsigned int HMC832_readReg3U8(unsigned char reg_num, unsigned int value, U8 *pGet) {
	unsigned char buffer[5];
	unsigned int * cmdstart = (unsigned int *) (&buffer[0]);
  	*cmdstart = ntohl((value << 8) | ((reg_num & 0x1F) << 3)); // << 3 for device address


  	HAL_SPI_Transmit(&HMC832_SPI_PORT, buffer, 1, 100);
  	DWT_Delay_us(1);
  	HMC832_SEN_Clr();
  	HAL_SPI_Transmit(&HMC832_SPI_PORT, buffer+1, 3, 100);
  	DWT_Delay_us(1);    // 출력 언뮤트
    HMC832_writeReg(0x20, 0x000000);
    HMC832_writeReg(0x60, 0x000000);

    // INT=48 (0x30), FRAC=0
    HMC832_writeReg(0x03, 0x000030);
    HMC832_writeReg(0x04, 0x000000);

    // R divider = 1 (예시: 레지스터 0x02)
    HMC832_writeReg(0x02, 0x000001);

    // Output divider = /8 (0x03 or 0x08 depending on 맵)
    HMC832_writeReg(0x0C, 0x000003);

    // Auto-cal trigger
    HMC832_writeReg(0x0E, 0x000001);
  	HMC832_SEN_Set();
  	DWT_Delay_us(1);

	//HAL_SPI_Transmit(&HMC832_SPI_PORT, buffer, 4, 100);
      	uint16_t NumByteToRead = 0x04;
    	uint8_t *pBuffer=buffer;
    	uint8_t counter=0;
    	while(NumByteToRead > 0x00){
    		// send dummy byte (0x00) to generate SPI clock to GYROSCOPE slave device
    		HAL_SPI_Receive(&HMC832_SPI_PORT, pBuffer, 1, 100);
    		if(counter == 0)
    		{
    		  	HMC832_SEN_Clr();
    		  	DWT_Delay_us(1);
    		}


    		NumByteToRead--;
    		pBuffer++; // MSB first
    		counter++;
    	}


    	DWT_Delay_us(1);
      	HMC832_SEN_Set();

	// MSB first to LSB fist 
	*pGet = buffer[3]; // anaCal[ch][iter][0] LSB
	pGet++; 
	*pGet = buffer[2]; // anaCal[ch][iter][1]
	pGet++;
	*pGet = buffer[1]; // anaCal[ch][iter][2] MSB, MSB:0

	// LSB first
      	unsigned int readResultShifted = (buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + (buffer[3] << 0);
      	unsigned int readRegValue = (readResultShifted >> 8);
      	return readRegValue;
}






/*
void HMC832_writeReg(unsigned char reg_num, unsigned int value) {
      	unsigned char buffer[5];
      	unsigned int * cmdstart = (unsigned int *) (&buffer[0]);
      	*cmdstart = ntohl((value << 8) | ((reg_num & 0x1F) << 3)); // << 3 for device address
      	//wiringPiSPIDataRW(0, buffer, 4);

      	HAL_SPI_Transmit(&HMC832_SPI_PORT, buffer, 1, 100);

      	HMC832_SEN_Clr();
      	HAL_SPI_Transmit(&HMC832_SPI_PORT, buffer+1, 3, 100);
    	__NOP();
      	HMC832_SEN_Set();

}
*/

/*

// ----------------------
// 기본 SPI 전송 함수 (24비트)
// ----------------------
static inline void HMC832_Write24(uint8_t b0, uint8_t b1, uint8_t b2) {
    HMC832_SEN_Clr();
    HAL_SPI_Transmit(&HMC832_SPI_PORT, &b0, 1, 100);
    HAL_SPI_Transmit(&HMC832_SPI_PORT, &b1, 1, 100);
    HAL_SPI_Transmit(&HMC832_SPI_PORT, &b2, 1, 100);
    HMC832_SEN_Set();
}

// ----------------------
// 레지스터 쓰기
// ----------------------
void HMC832_writeReg(uint8_t reg_num, uint32_t value)
{
    uint8_t buffer[4];
    uint32_t cmd = ((value & 0x00FFFFFF) << 8) | ((reg_num & 0x1F) << 3);
    // HMC832는 MSB first, 네가 쓰던 ntohl 매크로는 그대로 써도 되고
    buffer[0] = (cmd >> 24) & 0xFF;
    buffer[1] = (cmd >> 16) & 0xFF;
    buffer[2] = (cmd >>  8) & 0xFF;
    buffer[3] = (cmd >>  0) & 0xFF;

    HMC832_SEN_Clr();
    // 짧은 setup time이 필요하면 몇 ns~us 지연
    HAL_SPI_Transmit(&HMC832_SPI_PORT, buffer, 4, 100);
    HMC832_SEN_Set();
}
// ----------------------
// 레지스터 읽기
// ----------------------
 *
 */

void HMC832_writeReg(uint8_t reg, uint32_t val) {
    uint32_t frame = (val << 8) | ((reg & 0x1F) << 3); // 기존 로직 유지
    uint8_t buf[4];
    buf[0] = (frame >> 24) & 0xFF; // MSB first
    buf[1] = (frame >> 16) & 0xFF;
    buf[2] = (frame >> 8)  & 0xFF;
    buf[3] = (frame >> 0)  & 0xFF;

    HMC832_SEN_Clr();
    HAL_SPI_Transmit(&HMC832_SPI_PORT, buf, 4, 100);
    HMC832_SEN_Set();
    // 소량의 딜레이 권장
}

void HMC832_vspiWriteReg(uint8_t reg, uint8_t vcoreg, uint32_t val) {
	 uint32_t val1 = (val << 7) | ((vcoreg & 0xF) << 3);
    uint32_t frame = (val1 << 8) | ((reg & 0x1F) << 3); // 기존 로직 유지
    uint8_t buf[4];
    buf[0] = (frame >> 24) & 0xFF; // MSB first
    buf[1] = (frame >> 16) & 0xFF;
    buf[2] = (frame >> 8)  & 0xFF;
    buf[3] = (frame >> 0)  & 0xFF;

    HMC832_SEN_Clr();
    HAL_SPI_Transmit(&HMC832_SPI_PORT, buf, 4, 100);
    HMC832_SEN_Set();
    // 소량의 딜레이 권장
}

void HMC832_vspiWrite(uint8_t vco_reg, uint32_t value)
{
    // Step 1: VCO data 준비 (상위 17비트 데이터 + 4비트 주소)
    uint32_t vco_frame = ((value & 0x1FFFF) << 4) | (vco_reg & 0xF);

    // Step 2: PLL_REG 0x0E에 쓰기
    HMC832_writeReg(0x0E, vco_frame);

    // Step 3: PLL_REG 0x0F 트리거 (WRITE_EN=1)
    HMC832_writeReg(0x0F, 0x000001);
}


uint32_t HMC832_readReg(uint8_t reg) {
    uint32_t cmd = (0 /*read frame per datasheet*/ << 8) | ((reg & 0x1F) << 3);
    uint8_t tx[4] = {
        (cmd >> 24) & 0xFF, (cmd >> 16) & 0xFF,
        (cmd >> 8) & 0xFF,  (cmd >> 0)  & 0xFF
    };
    uint8_t rx[4];

    // 주소 프레임 전송
    HMC832_SEN_Clr();
    HAL_SPI_Transmit(&HMC832_SPI_PORT, tx, 4, 100);
    HMC832_SEN_Set();

    // 짧은 tSENH 후 데이터 읽기
    HMC832_SEN_Clr();
    HAL_SPI_Receive(&HMC832_SPI_PORT, rx, 4, 100);
    HMC832_SEN_Set();

    return ((uint32_t)rx[0]<<24)|((uint32_t)rx[1]<<16)|((uint32_t)rx[2]<<8)|rx[3];
}


