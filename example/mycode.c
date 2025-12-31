#include <math.h>
#include <stdio.h>
#include <stdint.h>
//#include "stm32f4xx_hal.h"


// variable redefinition
typedef unsigned char BOOL;
// no need with compiler_defs.h include
typedef uint8_t U8;
typedef uint16_t U16;
typedef uint32_t U32;
typedef char S8;
typedef int S16;
typedef long S32;
#define BYTE3 1  // for 3 bytes data
#define BYTE2 1  // for 2 bytes data


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


// 외부 16 MHz ref clock
#define HMC832_FREF   16000000UL

// extern SPI_HandleTypeDef hspi1;  // 실제 사용 포트

// -----------------------------
// SPI write / read
// -----------------------------
// 숫자를 2진수로 출력 (4bit씩 끊어서)
void print_binary(uint32_t num) {
    int bits = sizeof(num) * 8;  // 32bit 기준
    for (int i = bits - 1; i >= 0; i--) {
        printf("%d", (num >> i) & 1);  // 해당 비트 출력
        if (i % 4 == 0 && i != 0) {
            printf(" ");  // 4비트 단위로 띄어쓰기
        }
    }
    printf("\n");
}
void print_bits(unsigned char *arr, int size) {

    for (int i = 0; i < size; i++) {
        printf("0x%02x, ", arr[i]);
    }
    for (int i = 0; i < size; i++) {
        unsigned char byte = arr[i];
        for (int j = 7; j >= 0; j--) {
            printf("%d", (byte >> j) & 1);
            if (j % 8 == 0 && j != 0) {
                printf(" ");  // 4비트 단위로 구분
            }
        }
        printf(" ");  // 바이트 단위 구분
    }
    printf("\n");
}

void HMC832_writeReg(uint8_t reg, uint32_t val) {
    uint32_t frame = (val << 8) | ((reg & 0x1F) << 3);
    uint8_t buf[4];
    buf[0] = (frame >> 24) & 0xFF;
    buf[1] = (frame >> 16) & 0xFF;
    buf[2] = (frame >>  8) & 0xFF;
    buf[3] = (frame >>  0) & 0xFF;

    print_bits(buf, 4);
    //HMC832_SEN_Clr();
    //HAL_SPI_Transmit(&hspi1, buf, 4, 100);

    //HMC832_SEN_Set();
}
void HMC832_vspiWriteReg(uint8_t reg, uint8_t vcoreg, uint32_t val) {
	 uint32_t val1 = (val << 7) | ((vcoreg & 0xF) << 3);
    uint32_t frame = (val1 << 8) | ((reg & 0x1F) << 3); // 기존 로직 유지
    uint8_t buf[4];
    buf[0] = (frame >> 24) & 0xFF; // MSB first
    buf[1] = (frame >> 16) & 0xFF;
    buf[2] = (frame >> 8)  & 0xFF;
    buf[3] = (frame >> 0)  & 0xFF;

    print_bits(buf, 4);
    /*
    HMC832_SEN_Clr();
    HAL_SPI_Transmit(&HMC832_SPI_PORT, buf, 4, 100);
    HMC832_SEN_Set();
    */
    // 소량의 딜레이 권장
}

uint32_t HMC832_readReg(uint8_t reg) {
    uint32_t frame = (1 << 31) | ((reg & 0x1F) << 3); // read frame
    uint8_t tx[4], rx[4];
    tx[0] = (frame >> 24) & 0xFF;
    tx[1] = (frame >> 16) & 0xFF;
    tx[2] = (frame >>  8) & 0xFF;
    tx[3] = (frame >>  0) & 0xFF;

    print_bits(tx, 4);
    /*
    HMC832_SEN_Clr();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 4, 100);
    HMC832_SEN_Set();
    */

    return ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | (rx[2]);
}


unsigned int HMC832_readReg3U8(unsigned char reg_num, unsigned int value, U8 *pGet) {
	unsigned char buffer[5];
	unsigned int * cmdstart = (unsigned int *) (&buffer[0]);
  	*cmdstart = ntohl((value << 8) | ((reg_num & 0x1F) << 3)); // << 3 for device address

    /*
  	HAL_SPI_Transmit(&HMC832_SPI_PORT, buffer, 1, 100);
  	DWT_Delay_us(1);
  	HMC832_SEN_Clr();
  	HAL_SPI_Transmit(&HMC832_SPI_PORT, buffer+1, 3, 100);
  	DWT_Delay_us(1);
  	HMC832_SEN_Set();
  	DWT_Delay_us(1);
    */
    print_bits(buffer, 4);

    uint16_t NumByteToRead = 0x04;
    uint8_t *pBuffer=buffer;
    uint8_t counter=0;
    /*
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
   */

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




void HAL_Delay(unsigned int aa) { ; }

// -----------------------------
// 초기화 시퀀스 (예전에 쓰시던 배열 기반)
// -----------------------------
unsigned int initvals[] = {
    0x000020, 0x000002, 0x000010, 0x000000, 0x002000,
    0x000110, 0x000010, 0x000000, 0x002000, 0x000C98,
    0x000010, 0x000000, 0x002000, 0x004D38, 0x000010,
    0x000000, 0x002000, 0x000F4A, 0x000006, 0x000000,
    0x0007CC, 0x002047, 0x000008, 0x000002, 0x000000,
    0x000398, 0x000003, 0x000000, 0x000000, 0x000012,
    0x000000, 0x0027CD, 0x000012, 0x000000, 0x0027CD,
    0x000012, 0x000000, 0x0027CD, 0x000012, 0x000000,
    0x0027CD, 0x000012, 0x0023CD, 0x000012, 0x000000
};
unsigned char initregs[] = {
    0x00, 0x01, 0x00, 0x00, 0x05,
    0x05, 0x00, 0x00, 0x05, 0x05,
    0x00, 0x00, 0x05, 0x05, 0x00,
    0x00, 0x05, 0x06, 0x00, 0x00,
    0x07, 0x0A, 0x02, 0x00, 0x00,
    0x03, 0x00, 0x00, 0x04, 0x00,
    0x00, 0x07, 0x00, 0x00, 0x07,
    0x00, 0x00, 0x07, 0x00, 0x00,
    0x07, 0x00, 0x07, 0x00, 0x00
};


unsigned char setregs[] = { 0x03, 0x0C, 0x04 };
unsigned int setvals[] = { 0x000000, 0x000000, 0x000000 };


unsigned char setFractRegs[] = { 0x02, 0x03, 0x04 };
unsigned int setFractVals[] = { 0x000000, 0x000000, 0x000000 };


unsigned char setregsnoauto[] = { 0x03, 0x0C, 0x04, 0x05 };

unsigned int setvalsnoauto[] = { 0x000000, 0x000000, 0x000000, 0x000000 };
        


// -----------------------------
// 주파수 설정 함수
// -----------------------------
void HMC832_SetFrequency(double fout)
{
    uint32_t outdiv;
    double fvco;
    uint32_t int_val, frac_val;
    uint32_t cur, divsel;

    // 1. OUTDIV 결정
    if (fout < 250e6)       outdiv = 32;
    else if (fout < 500e6)  outdiv = 16;
    else if (fout < 1000e6) outdiv = 8;
    else if (fout < 2000e6) outdiv = 4;
    else if (fout < 4000e6) outdiv = 2;
    else                    outdiv = 1;

    // 2. VCO 목표 주파수
    fvco = fout * outdiv;

    // 3. INT / FRAC 계산
    double N = fvco / (double)HMC832_FREF;
    int_val  = (uint32_t)floor(N);
    frac_val = (uint32_t)round((N - int_val) * (1ULL << 24));

    // 4. 레지스터 적용
    printf("outdiv , %d\n", outdiv);
    printf("write 0x03, 0x%06x\n", int_val);
    HMC832_writeReg(0x03, int_val & 0xFFFFF);

    printf("write 0x04, 0x%06x\n", frac_val);
    HMC832_writeReg(0x04, frac_val & 0xFFFFFF);

    switch(outdiv) {
        case 1: divsel = 0; break;
        case 2: divsel = 1; break;
        case 4: divsel = 2; break;
        case 8: divsel = 3; break;
        case 16: divsel = 4; break;
        case 32: divsel = 5; break;
        case 64: divsel = 6; break;
        case 128: divsel = 7; break;
    }

    printf("read 0x0C\n");
    cur = HMC832_readReg(0x0C);
    cur &= ~0x7;
    cur |= (divsel & 0x7);

    printf("write 0x0C, 0x%06x\n", cur);
    HMC832_writeReg(0x0C, cur);

    // 5. Calibration 트리거

    printf("write 0x05\n");
    printf("write 0x05, 0x%06x\n", 0x000000);
    HMC832_writeReg(0x05, 0x000000);
    HAL_Delay(1);

    printf("write 0x05, 0x%06x\n", 0x002000);
    HMC832_writeReg(0x05, 0x002000);
    HAL_Delay(10);
}


unsigned int hcf(unsigned int n1, unsigned int n2) {
	if (n2 != 0)
		return hcf(n2, n1 % n2);
	else
		return n1;
}


void HMC832_SetFrequency1(double fout)
{

    unsigned char ndivider, outdivider, rdivider=0x08, kval;
    unsigned int reg0C, reg03, reg04, prereg03;
    float fpd, fxtal, fNcurr;
    U8 anaCal;
    U8 *anaCalPtr= &anaCal;

	if (fout/1e6 < 1500.0) {
		for (int i = 62; i >= 2; i -= 2) {
			if ((fout/1e6 * i) < 3000.0) {
				ndivider = i;
				break;
			}
		}
	}
	outdivider = ndivider * rdivider; //2x8=16
	kval = ndivider; //2
	fxtal = 16e6;
	fpd = fxtal / rdivider; // 2,000,000
	// fgcdmin = fpd / pow(2, 14);

    reg03 = floor(fout * kval / fpd);
    printf("\nkval :%d\n\n",   kval);
    printf("\nfGCD :%d\n\n",  (hcf(fpd, fout * kval)));
	reg0C = fpd / (hcf(fpd, fout * kval));
	fNcurr = floor(fout * kval / fpd) * fpd;

	setvals[0] = reg03;
	setvals[1] = 0x000000;


	reg04 = (pow(2, 24) * (fout * kval - fNcurr) / fpd);
	setvals[1] = reg0C;
	setvals[2] = reg04;

    printf("write 0x%02x, 0x%06x,  \n\n", 0x03, reg03);
    printf("write 0x%02x, 0x%06x,  \n\n", 0x0C, reg0C);
    printf("write 0x%02x, 0x%06x,  \n\n", 0x04, reg04);

	//setvals[1] = reg0C;
	// set HMC832 frequency register 03//
	// initialize HMC832 //
	for (int j = 0; j < 24; j++) {
		if(j <3 || j == 4 || j == 6 || j == 8 || j == 10 || j == 12  ) // N freq set, Freq fraction set, 0x07 lock detector
		{

            printf("write 0x%02x, 0x%06x,  ", setregs[j], setvals[j]);
			HMC832_writeReg(setregs[j], setvals[j]);
		} else if(j == 3 || j == 5 || j==7 || j== 9 || j== 11 || j == 13 || j == 14 || j == 15 ) // 0x07 read lock ( table 21)
		{
            printf("write 0x%02x, 0x%06x,  ", setregs[j], setvals[j]);
			HMC832_readReg3U8(setregs[j], setvals[j], anaCalPtr);
		}

		if (j > 2)
			HAL_Delay(1);
		else
			HAL_Delay(1);
	}


	setvalsnoauto[2] = 0x002000 | ((0x00FF00 & (anaCal << 9)) | (0x00FF00 & (anaCal << 1))); // D[7:0] => D[15:7] D[13]=1
	for (int j = 0; j < 4; j++) {
		if( j < 3 )
		{

            printf("write 0x%02x, 0x%06x,  ", setregs[j], setvals[j]);
			HMC832_writeReg(setregs[j], setvals[j]);
		} else
		{

            printf("write 0x%02x, 0x%06x,  ", setregsnoauto[j], setvalsnoauto[j]);
			HMC832_writeReg(setregsnoauto[j], setvalsnoauto[j]);
		}
	}
	



}

// HMC832 상수 정의
#define VCO_MIN_FREQ_MHZ    1500.0f
#define VCO_MAX_FREQ_MHZ    3000.0f
#define PLL_REG_0C_ADDR     0x0C
#define SPI_WRITE_BIT_POS   31
// -------------------------------------
// PLL 주파수 설정 함수
// -------------------------------------
void HMC832_SetFractFrequency1(double fout, double us)
{

    unsigned char outdivider, rdivider=0x01, kval;
    unsigned int reg02, reg03, reg04, prereg03;
    unsigned int fpd, fxtal, nFRACT, nINT;
    float nTOT;
    U8 anaCal;
    U8 *anaCalPtr= &anaCal;
    reg02 = 0x01;
    kval = 0x01;

    float d_min_ideal = VCO_MIN_FREQ_MHZ / (fout/1e6);

    /*
	if (fout/1e6 < 1500.0) {
		for (int i = 62; i >= 2; i -= 2) {
			if ((fout/1e6 * i) < 3000.0) {
				kval = i;
				break;
			}
		}
	}
	*/

	if (fout/1e6 >= VCO_MIN_FREQ_MHZ && fout/1e6 <= VCO_MAX_FREQ_MHZ) {
        kval = 1;
    } else {
        // D=2, 4, 6, ..., 62 케이스 확인
        for (int i = 1; i <= 31; i++) {
            uint8_t current_D = (uint8_t)(i * 2); // 2, 4, 6, ..., 62
            printf("i %d\n", i);
            if ((float)current_D >= d_min_ideal) {
                kval = current_D;
                printf("kval %d\n", kval);
                break;
            }
        }
    }

	outdivider = kval * rdivider; //2x1=16
	fxtal = 16e6;
	fpd = fxtal / rdivider; // 2,000,000
	// fgcdmin = fpd / pow(2, 14);

	nTOT = fout * kval / fpd;

	reg02 = rdivider;
    reg03 = floor(nTOT);
    nFRACT =   (unsigned int)(nTOT-reg03)*pow(2,24);
    if (nFRACT > 20 ) {
    	reg04 = nFRACT;
    }
    else {
    	reg04 = 20;
    }
	setFractVals[0] = reg02;
	setFractVals[1] = reg03;
	setFractVals[2] = reg04;




    printf("write 0x%02x, 0x%06x,  \n\n", 0x02, reg02);
    printf("write 0x%02x, 0x%06x,  \n\n", 0x03, reg03);
    printf("write 0x%02x, 0x%06x,  \n\n", 0x04, reg04);

	for (int j = 0; j < 3; j++) {
		HMC832_writeReg(setFractRegs[j], setFractVals[j]);
    	HAL_Delay(us);
	}

    printf("write 0x%02x, 0x%02x, 0x%06x,  \n\n", 0x05, 0x02, kval);
	HMC832_vspiWriteReg(0x05, 0x02, kval);
    HAL_Delay(us);
}




// -----------------------------
// 전체 초기화 + 주파수 설정
// -----------------------------
void HMC832_InitAndSetFreq(double fout)
{
    // 초기화 루프
    printf("Initialize:\n");
    for (int i = 0; i < 45; i++) {

        printf("write 0x%02x, 0x%06x,  ", initregs[i], initvals[i]);
        HMC832_writeReg(initregs[i], initvals[i]);
        HAL_Delay(1);
    }

    HAL_Delay(10); // 초기화 안정화 대기

    // 원하는 주파수 세팅

    printf("Frequency set:\n");
    HMC832_SetFrequency(fout);
}


main() {

    /*
    printf("Freq\n");
    HMC832_SetFrequency(300.0e6);

    printf("Freq1\n");
    HMC832_SetFrequency1(300.0e6);
    */

    printf("Freq 300\n");
    HAL_Delay(5000);
    HMC832_SetFractFrequency1(300e6, 1);
    HAL_Delay(5000);

    printf("Freq 200MHz\n");
    HMC832_SetFractFrequency1(200e6, 1);
    HAL_Delay(5000);

    printf("Freq 400MHz\n");
    HMC832_SetFractFrequency1(400e6, 1);
    HAL_Delay(5000);

}
