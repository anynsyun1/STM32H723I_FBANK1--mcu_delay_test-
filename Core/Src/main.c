/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "includes.h"
#include "dwt_stm32_delay.h"

#define MAXAVG  8
//#define POINTERON 1

uint16_t adc_buff[MAXAVG+1];
uint16_t user_buff[MAXAVG+1];

volatile uint32_t ADC_ConvCpltFlag;
volatile uint32_t ADC_HalfConvCpltFlag;
volatile uint16_t ADC_monitor;

uint32_t maxval;



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


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

#define MASTERID 1


/////////////////////////////////////////////////////////////////////////
// RS485 communication control definition
//U8 RxData[MAX485], TxData[MAX485];
/////////////////////////////////////////////////////////////////////////
#define MAX485 256// as bytes
//#define HEAD485 6 // as bytes

static uint8_t myID = 0;

///////////////////////////////////////////////////////////////
/// header and data item location on the Rx frame data RxData
//  on RS485
#define IDLOC 0
#define FNLOC 1
#define ADLOC 2
#define CNLOC 4
#define CRCSZ 2

//////////////////////////////////////////////////////////////
/// RS485 address range and func code definition
#define READ_DIST 2
#define READ_HOLD 3
#define READ_INPUT 4
#define WRITE_SINGLE 6
#define WRITE_MULTIPLE 16

#define READ_DIST_IDX 0
#define READ_HOLD_IDX 1
#define READ_INPUT_IDX 2
#define WRITE_SINGLE_IDX 3
#define WRITE_MULTIPLE_IDX 4

/////// x68 function control
#define FUNCLIST 5 // max function list for x68
// 0x01 : Illegal function
// 0x02 Illegal address
// 0x03 Illegal data value
// 0x04 slave device failure
// 0x05 acknowledge , but processing
// 0x06 slave is busy
#define ILLEGAL_FUNC  0x01
#define ILLEGAL_ADDR  0x02
#define ILLEGAL_VALUE 0x03
#define SLAVE_FAILURE 0x04
#define PROCESSING    0x05
#define SLAVE_BUSY    0x06

///////////////////////////////////////////////////////////////////
///////////// address index map to read from main for loop ////////
#define REG_SIZE 60
//////////////////////////////////////////////////////////////////////
/////// x68 command control flag  /////////////////////////////////
// messageing block, not using
/*
*/
#define RD_BLOCK        0x02

// execution block, using
#define MASS_BLOCK      0x03

#define X68_BLOCK       0x04
#define CA_BLOCK        0x05
#define PAUSE_BLOCK     0x06
#define DBG_BLOCK       0x07
#define EE_ID_BLOCK     0x08
#define MEAS_BLOCK      0x09
#define NO_MEAS_BLOCK   0x0A

// control block
#define DF_BLOCK        0x0B



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#include"hmc832spi_stm32.h"
#include"pe43712_spi_stm32.h"

// #include"stm32_eeprom_func1.h" // This should be uncomment later
// https://community.st.com/s/question/0D50X00009XkXbw/stm32h7-hal-flash-program

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

#define MAXSIZE 256
#define AVGSIZE 70

#define NUM_SIZE6 6
#define NUM_SIZE8 8
#define NUM_SIZE16 16
#define CHANN 3

#define MAX_PRECISION   (10)

#define I2C_WRITE_CMD 0
#define I2C_PORT hi2c1

#define MAX_ANGLE 3.3
#define NUM_SIZE16 16
#define CHANN 3




#if defined( __ICCARM__ )
#define DMA_BUFFER \
_Pragma("location=\".dma_buffer\"")
#else
#define DMA_BUFFER \
__attribute__((section(".dma_buffer")))
#endif


unsigned int initvals[] = { 0x000020, 0x000002, 0x000010, 0x000000, 0x002000,
	0x000110, 0x000010, 0x000000, 0x002000, 0x000C98, 0x000010, 0x000000,
	0x002000, 0x004D38, 0x000010, 0x000000, 0x002000, 0x000F4A, /* FRAC mode [11]=1, [7]=0 */ 0x000006,
	0x000000, 0x0007CC, 0x002047, 0x000001, /* r divider */ 0x000002, 0x000000, 0x000398,
	0x000003, 0x000000, 0x000000, 0x000012, 0x000000, 0x0027CD, 0x000012,
	0x000000, 0x0027CD, 0x000012, 0x000000, 0x0027CD, 0x000012, 0x000000,
	0x0027CD, 0x000012, 0x0023CD, 0x000012, 0x000000 };
unsigned char initregs[] = { 0x00, 0x01, 0x00, 0x00, 0x05, 0x05, 0x00, 0x00,
	0x05, 0x05, 0x00, 0x00, 0x05, 0x05, 0x00, 0x00, 0x05, 0x06, 0x00, 0x00,
	0x07, 0x0A, 0x02, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0x00, 0x00, 0x07,
	0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00,
	0x00 };



unsigned int initauto=0x002047;
unsigned int initnoauto=0x002847;

unsigned int initvalsauto[] = { 0x000020, 0x000002, 0x000010, // 2, Reset RST, PLL bit enable
	0x002000, 0x000110, 0x000010, // 5, VCO reg 0x02 R=2 setting
	0x002000, 0x000C98, 0x000010, // 8,  VCO reg 0x03 settubg
	0x002000, 0x004d38, 0x000010, // 11, VCO reg 0x00 setting, vco cal voltage
	0x002000, 0x000F4A, 0x000006, // 14, VCO reg 0x00 enable fractional mode
	0x0007CC, 0x002047, 0x000008, 0x000002, // 18, VCO reg 0x03 setting
	0x000398, 0x000003, // 20, 0x03 n divider set
	0x000000, 0x000012, // 22, 0x04 fractional freq set
	0x0027CD, 0x000012, // 24,
	0x0027CD, 0x000012, // 26,
	0x0027CD, 0x000012, // 28,
	0x0027CD, 0x000012, // 30
	0x0023CD, 0x000012 // 32
};

// 0x004b38 ( 6db gain), 0x004db8(11db gain )

unsigned char initregsauto[] = { 0x00, 0x01, 0x00,  // Reset RST, PLL bit enable
	0x05, 0x05, 0x00, // VCO reg 0x02 R=2 setting
	0x05, 0x05, 0x00, // VCO reg 0x03 settubg
	0x05, 0x05, 0x00, // VCO reg 0x00 setting, vco cal voltage
	0x05, 0x06, 0x00, // VCO reg 0x00 enable fractional mode
	0x07, 0x0A, 0x02, 0x00,  // VCO reg 0x03 setting
	0x03, 0x00, // 0x03 n divider set
	0x04, 0x00, // 0x04 fractional freq set
	0x07, 0x00,
	0x07, 0x00,
	0x07, 0x00,
	0x07, 0x00,
	0x07, 0x00
};

unsigned char initautocaloff[] = { 0x0A, 0x00 };
unsigned int setautocaloff[] = { 0x002847, 0x00000A };

/*
*/

unsigned char setregs[] = { 0x03, 0x0C, 0x04 };
unsigned int setvals[] = { 0x000000, 0x000000, 0x000000 };

unsigned char setFractRegs[] = { 0x02, 0x03, 0x04 };
unsigned int setFractVals[] = { 0x000000, 0x000000, 0x000000 };

/*
*/

unsigned char setregsnoauto[] = { 0x03, 0x0C, 0x04, 0x05 };

unsigned int setvalsnoauto[] = { 0x000000, 0x000000, 0x000000, 0x000000 };

unsigned char onoffregs[] = { 0x05, 0x05, 0x05, 0x05 };
unsigned int onoffvals[] = { 0x002000, 0x004D38, 0x002000, 0x00CD38 };


unsigned char onregs[] = { 0x05, 0x05 };
unsigned int onvals[] = { 0x002000, 0x000C98 };
unsigned char offregs[] = { 0x05, 0x05 };
unsigned int offvals[] = { 0x002000, 0x000C18 };


// 0x2000 0x4D38 : output on, 0x2000 0xCD38 out off

/*
 *
*/


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

volatile uint32_t ADC_ConvCpltFlag;
volatile uint32_t ADC_HalfConvCpltFlag;


/////////////// RS485 Variable //////////////////////////
////////////////////////////////////////////////////
/// TESTER VARIABLE
////////////////////////////////////////////////////
#define MAXINDX  2000 // 125 //1024 // not over 999, 2500 for stm32h723

#define MAX3BYTS 29 // 242: 4+3+30*3+1 // over 30, it is not working
#define MAX2BYTS 24 // 251: 4+2+??*2+1


volatile uint32_t ADC_ConvCpltFlag;
volatile uint32_t ADC_HalfConvCpltFlag;
volatile uint16_t ADC_monitor;

#define MAXLOOP 1
#define MAXAVGLOOP 6
#define MAXCH 11


//////////////////////////////////////////////////////
static U8 firstLoop=1;

//////////////////////////////////////////////////////

uint32_t valTot[MAXLOOP];

uint32_t anaValTot[MAXINDX];
uint32_t anaValFltd[MAXINDX];
uint32_t anaValPreAvg[MAXCH][MAXINDX];
float anaVal[MAXINDX];


uint32_t anaValTot1[MAXINDX];
uint32_t anaValFltd1[MAXINDX];
uint32_t anaValPreAvg1[MAXCH][MAXINDX];
float anaVal1[MAXINDX];


// end of attenuator value control


float measedFreq[MAXCH];
static U8 vergine=1;
static U8 preCh;


float peakFreq = 0.0;
U16 peakVal = 0;
/// get max frequency
uint32_t maxfreqval = 0;
uint32_t maxfreqidx = 0;


unsigned char outputdivider = 1;
const unsigned char rdivider = 8;
unsigned char ndivider;



////////////////////////////////////////////////
// cal value to host
U8 anaCal[MAXCH][MAXINDX][3];
U8 calVal[3];


//////// cont avererage //////
uint32_t measAvg[MAXAVGLOOP];

#define MAXCALIDX  3 // CALIBRATION
#define TOTIDX  0 // TOT
#define SUMIDX  1 // SUM
#define FLTIDX  2 // FLTED
#define PREIDX  3 // PRE FLTED


typedef struct FQNODE{
	uint32_t idx;
	U8 calVal[MAXCALIDX];
	uint32_t anaVal[PREIDX+1];	
	struct FQNODE *next;
}FREQNODE;

FREQNODE *fqhead[MAXCH] = { NULL, NULL, NULL, NULL, NULL, \
	NULL, NULL, NULL, NULL, NULL, NULL};

//FREQNODE *fqhead=NULL;

FREQNODE *addFirstF(FREQNODE *target, uint32_t idx)
{
	FREQNODE *newNode = malloc(sizeof(FREQNODE));
	newNode->next = target;
	newNode->idx = idx;
	for(int i=0; i <= PREIDX; i++)
	{
		newNode->anaVal[i] = 0;
	}
	target = newNode;
	return newNode;
}


void addLastF(FREQNODE *target, uint32_t idx)
{

	FREQNODE *curr= target;
	while(curr->next != NULL)
	{
		curr = curr->next;
	}
	FREQNODE *newNode = malloc(sizeof(FREQNODE));
	curr->next = newNode;
	newNode->next = NULL;

	newNode->idx = idx;
	for(int i=0; PREIDX >= i; i++)
	{
		newNode->anaVal[i] = 0;
	}

}


void updateValF(FREQNODE *target, uint32_t idx, U8 whatis, uint32_t val)
{
	FREQNODE *curr= target;
	while(curr->next != NULL)
	{
		if(curr->idx == idx)
		{
			if( PREIDX == whatis )
			{
				curr->anaVal[whatis] = (uint32_t)(val*0.5 + curr->anaVal[whatis]*0.5);  
			}
			if( PREIDX > whatis)
			{
				curr->anaVal[whatis] = val;
				break;
			}
		}
		curr = curr->next;
	}
}

void updateCalF(FREQNODE *target, uint32_t idx, U8 *val)
{
	FREQNODE *curr= target;
	while(curr->next != NULL)
	{
		if(curr->idx == idx)
		{
			curr->calVal[0] = *(val+0);
			curr->calVal[1] = *(val+1);
			curr->calVal[2] = *(val+2);
			break;
		}
		curr = curr->next;
	}
}

U8 getCalF(FREQNODE *target, uint32_t idx, U8 whatis)
{
	FREQNODE *curr= target;
	if(MAXCALIDX > whatis )
	{	
		while(curr->next != NULL)
		{
			if(curr->idx == idx)
			{
				return curr->calVal[whatis];
			}
			curr = curr->next;
		}
	}
	return 0;
}



uint32_t getValF(FREQNODE *target, uint32_t idx, U8 whatis)
{
	FREQNODE *curr= target;
	while(curr->next != NULL)
	{
		if(curr->idx == idx)
		{
			return curr->anaVal[whatis];
		}
		curr = curr->next;
	}
	return 0;
}


void deleteF(FREQNODE *target)
{
	FREQNODE *curr = target->next;      // 연결 리스트 순회용 포인터에 첫 번째 노드의 주소 저장
	while (curr != NULL)    // 포인터가 NULL이 아닐 때 계속 반복
	{
		FREQNODE *next = curr->next;    // 현재 노드의 다음 노드 주소를 임시로 저장
		free(curr);                        // 현재 노드 메모리 해제
		curr = next;                       // 포인터에 다음 노드의 주소 저장
	}
	free(target);    // 머리 노드 메모리 해제
	target->next = NULL;
	target = NULL;
}

typedef struct FLTNODE {
	U8       order;
	U8       ampl;
	U8       cnt; // middle of kernel ticket
	FREQNODE *freqptr;
	struct FLTNODE *next;
} FILTERNODE;

FILTERNODE *flthead[MAXCH] = { NULL, NULL, NULL, NULL, NULL, \
	NULL, NULL, NULL, NULL, NULL, NULL};



void createFLT(FILTERNODE *target, uint32_t kernel)
{

	U8 center = 0;
	FILTERNODE *curr= target;
	curr->next = NULL;

	if( 0 == (kernel % 2))
	{
		kernel = kernel+1;
	}
	center = (kernel+1)/2;

	for(int i=1, l=0; i >= kernel; i++)
	{
		FILTERNODE *newNode = malloc(sizeof(FILTERNODE)); //새로운 노드생성
		newNode->next = curr;                             // 새로운 노드 끝에 현재 노드 연결
		curr = newNode;
		curr->order = i;
		if( i < center)
		{
			curr->ampl = i;
			curr->cnt=0;
		}
		else if( i == center )
		{
			curr->ampl = center;
			curr->cnt=1;
			l = center;
		}
		else if( i > center )
		{
			l--;
			curr->ampl = l;
			curr->cnt = 0;
		}
		curr->freqptr=NULL;
	}
}

void deleteFLT(FILTERNODE *target)
{
	FILTERNODE *curr = target->next;      // 연결 리스트 순회용 포인터에 첫 번째 노드의 주소 저장
	while (curr != NULL)    // 포인터가 NULL이 아닐 때 계속 반복
	{
		FILTERNODE *next = curr->next;    // 현재 노드의 다음 노드 주소를 임시로 저장
		free(curr);                        // 현재 노드 메모리 해제
		curr = next;                       // 포인터에 다음 노드의 주소 저장
	}
	free(target);    // 머리 노드 메모리 해제
	target->next = NULL;
	target = NULL;
}



void softfilter(FREQNODE *target, FILTERNODE *flt)
{
	// alocate memory
	FREQNODE *curr= target;
	FREQNODE *tmp1=NULL, *tmp2= NULL;
	FREQNODE *cntPtr=NULL;

	FILTERNODE *currFlt = flt;
	U8 isFirst = 1;

	double val1=0.0;

	while(curr->next != NULL) // FREQNODE scan
	{
		val1 = 0.0;
		currFlt = flt;
		while(currFlt->next != NULL) // FILTNODE scan 주파수노드를 하나씩 밀어 내린다.
		{
			tmp1 = currFlt->freqptr; // 현재 노드의 내용물 임시보관
			if(isFirst) // 처음에는 새로운 FREQNODE를 top에 가져온다
			{
				currFlt->freqptr = curr;
				isFirst = 0;
			} else { // 그다음부터는 내부적으로 아래로 한칸씩 이동
				currFlt->freqptr = tmp2; // 위에서 내려온것 현재노드에 넣어줌
			}
			tmp2 = tmp1; // 임시보관
			if(currFlt->cnt == 1)
			{
				cntPtr = currFlt->freqptr;
			}
			val1 += (currFlt->ampl * (currFlt->freqptr)->anaVal[SUMIDX] * 1.0);
			currFlt = currFlt->next; // next node move
		}


		if( val1 > 4294967295.0)
		{
			val1 = 4294967295/256.0;
			cntPtr->anaVal[FLTIDX] = val1;
		} else {
			cntPtr->anaVal[FLTIDX] = val1 / 256.0;
		}


		isFirst = 1;
		tmp1 = tmp2 = NULL;

		curr = curr->next; // next node move
	}

	currFlt = flt;	
	while(currFlt->next != NULL) // FILTNODE 내용 clear 
	{
		currFlt->freqptr=NULL;
		currFlt = currFlt->next;
	}

}



///////// channel & freq control variable


typedef struct CHNODE{
	U8 ch;
	float stFreq;
	float spFreq;
	struct CHNODE *next;
}NODE;

NODE *head=NULL;

uint8_t pullstart;

void addFirst(NODE *target, U8 ch, float st, float sp)
{
    	NODE *newNode = malloc(sizeof(NODE));
    	newNode->next = target->next;
    	newNode->ch = ch;
    	newNode->stFreq = st;
    	newNode->spFreq = sp;
    	target->next = newNode;
}

void addLast(NODE *target, U8 ch, float st, float sp)
{
	NODE *curr= target;
	while(curr->next != NULL)
	{
		curr = curr->next;
	}
    	NODE *newNode = malloc(sizeof(NODE));
    	curr->next = newNode;

    	newNode->next = NULL;
    	newNode->ch = ch;
    	newNode->stFreq = st;
    	newNode->spFreq = sp;
}

void delete(NODE *target)
{
	NODE *curr = target->next;      // 연결 리스트 순회용 포인터에 첫 번째 노드의 주소 저장
	while (curr != NULL)    // 포인터가 NULL이 아닐 때 계속 반복
	{
		NODE *next = curr->next;    // 현재 노드의 다음 노드 주소를 임시로 저장
		free(curr);                        // 현재 노드 메모리 해제
		curr = next;                       // 포인터에 다음 노드의 주소 저장
	}
	free(target);    // 머리 노드 메모리 해제
	target->next = NULL;
	target = NULL;

}



int division;
typedef struct CHANNODE{
	U8 ch;
	float stFreq;
	float spFreq;
	FREQNODE *fqNode[MAXCH];
	FILTERNODE *ftNode[MAXCH];
	struct CHANNODE *next;
} CHANNELNODE;

CHANNELNODE *chhead=NULL;


void addLastCH(CHANNELNODE *target, U8 ch, float st, float sp, uint32_t div)
{
	CHANNELNODE *curr= target;
	while(curr->next != NULL)
	{
		curr = curr->next;
	}
	CHANNELNODE *newNode = malloc(sizeof(CHANNELNODE));
	curr->next = newNode;

	newNode->next = NULL;
	newNode->ch = ch;
	newNode->stFreq = st;
	newNode->spFreq = sp;

	newNode->fqNode[ch] = fqhead[ch];
	newNode->ftNode[ch] = flthead[ch];
	fqhead[ch]->next = NULL;
	flthead[ch]->next = NULL;

		//addLastF(newNode->fqNode[ch], 10);

	for(int i=0; div > i; i++)
	{
		addLastF(newNode->fqNode[ch], i);
	}
	createFLT(newNode->ftNode[ch], 16);
}

void deleteCH(CHANNELNODE *target)
{
	CHANNELNODE *curr = target->next;      // 연결 리스트 순회용 포인터에 첫 번째 노드의 주소 저장
	while (curr != NULL)    // 포인터가 NULL이 아닐 때 계속 반복
	{
		CHANNELNODE *next = curr->next;    // 현재 노드의 다음 노드 주소를 임시로 저장

		deleteF(curr->fqNode[curr->ch]);
		deleteFLT(curr->ftNode[curr->ch]);
		curr->fqNode[curr->ch] = NULL;
		curr->ftNode[curr->ch] = NULL;
		free(curr);                        // 현재 노드 메모리 해제
		curr = next;                       // 포인터에 다음 노드의 주소 저장
	}

	free(target);    // 머리 노드 메모리 해제
	target->next = NULL;
	target = NULL;

}



static U32 valIndex = 0;
static U32 bufIndex = 0;

static U8 RxData[MAX485], TxData[MAX485];
//static char *pRxData, *pTxData;

U8 mydata[2];

//static char *pRxData, *pTxData;
U8 *pTxData;

static U8 RxFlag = 0;

U8 mydata[2];

//static U8 myID;
static U16 RxSize, RxDLen, RxAddr;
static U8 TxLen, RxID, FnCode, FnErrCode, ErrStatus;

int Rx_indx;
int rxiter;
char rxBuffer[MAXSIZE];
uint8_t Rx_Buffer[MAXSIZE], Transfer_cplt, uartit_on;
uint8_t Rx_data;
char buff_rx[MAXSIZE];

U32 test_var;
// it should be start from 00 to FF
// 0x01 : Illegal function
// 0x02 Illegal address
// 0x03 Illegal data value
// 0x04 slave device failure
// 0x05 acknowledge , but processing
// 0x06 slave is busy
//Explanation of error codes

/* not use on Sera
// 0x07 negative ack, the slave can not perform command
// 0x08 parity error
// 0x0A gateway path unavailable
// 0x0B gateway target device failed
*/

U8 rs485exceptCode[17] = { 0x00, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8F, 0x90 };

const U8 fcl[FUNCLIST] = { READ_DIST, READ_HOLD, READ_INPUT,
	WRITE_SINGLE, WRITE_MULTIPLE };

/// error
U16 adl[FUNCLIST][2] = { { READ_DIST * 1000, READ_DIST * 1001 }, { READ_HOLD
	* 1000, READ_HOLD * 1015 }, { READ_INPUT * 1000, READ_INPUT * 1023 }, {
		WRITE_SINGLE * 1000, WRITE_SINGLE * 1001 }, { 6004, 6007 } };

U8 valInAddr[REG_SIZE];

static volatile U8 blockCtrl = DF_BLOCK;


////////////////////////////////////////
float vdd;
uint32_t ADC_raw[4];
float temperature;

/// Calibration offset
U32 offsetCode[2] = { 0, 0 };


long period, period1, periodsum, periodsum1, pulsesum1, pulse, pulse1, pulsesum,
     pulsesum1;
int len, sumcnt, sumcnt1;
int avgperiod, avgperiod1, avgpulse, avgpulse1;

//////////////// ITF frequency setup //////////////////



float frequency;
float stfrequency;
float spfrequency;
float freqstep;
float vcofrequency;
unsigned int integerdivider, fractionaldivider;

int kval, division;
float fpd, fxtal;
float stfreq, spfreq;
float fvco1, fvco2, fvco3;
float fgcdmin;

unsigned int fgcd1, fgcd2, fgcd3;
unsigned int reg0C, reg03, reg04, prereg03;
unsigned int fvcoM1, fvcoM2, fvcoM3;

float spfreqM, stfreqM, curfreqM, fstepM;
//unsigned int spfreqM, stfreqM, curfreqM;

unsigned int fNcurrM, fNcurr;
unsigned int tmpval;

int iter;
U32 maxval, minval, avgval;
U32 maxidx, minidx;

//////////////////// temp variable
unsigned int pre = 0, valstep;

//const float freq = 32000000.0; // 32mhz
float freq_offset = 0, angle, angle1;
float freq_offset1 = 0;
////// display control variable /////////
uint8_t angDispOn = 1;
uint8_t timDispOn = 0;
uint8_t xferDispOn = 0;

////// X-fer control variable   /////////
uint8_t xferOn = 0;
uint16_t dataSize = 0;
uint16_t tmCount;

char lcddisp[NUM_SIZE8];
char angbuffer[CHANN][NUM_SIZE8 + 1];
char txbuffer[CHANN][NUM_SIZE8 + 4];

float data[256];


uint32_t reading1, reading2, reading3;






/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

void SPECIAL_MX_GPIO_Init(uint8_t);


#ifdef DBG_ON
#endif

unsigned char str[20];
unsigned char buffer[50];

unsigned char ubuffer[50];
unsigned char abuffer[50];

uint8_t MonBuf[MAXSIZE];
uint16_t MonIn;

uint8_t powerMeasOn;


// get POWER(dBm) from voltage level
float level2Power (float volt)
{
	float maxdbm = 0.0;
	float mindbm = -52.5;

	float maxlvl = 2.5;
	float minlvl = 1.0;

	float dbm;


	if(volt > 0.0) {
		dbm = ((maxdbm-mindbm)/(maxlvl-minlvl))*(volt-minlvl)+mindbm;
	} else {
		dbm = mindbm;
	}
	return dbm;
}

uint8_t pe43712_attn2code(float attn)
{
	uint8_t code;
	code = (uint8_t)(4*attn);
	return code;
}

float pe43712_code2attn(uint8_t code)
{
	float attnval;
	attnval = (float)(code/4);
	return attnval;
}


// vari attenuator level control
static int maxSensVal[MAXCH];
static uint8_t  currAttnVal[MAXCH];


#define MIDMAX   8802
#define MIDMIN   7602
/*
#define MIDMAX   2200
#define MIDMIN   1900
*/

static uint8_t highcode[MAXCH];
static uint8_t lowcode[MAXCH];
static uint8_t midcode[MAXCH], premidcode[MAXCH];




//////////////////////////////////////////////////////////
//
// volt is code level
uint8_t contBinSearch(int volt, int ch) {
	// under search process
	premidcode[ch] = midcode[ch];

	// 범위밖
	if (MIDMAX < volt || MIDMIN > volt) {

		// 뒤바뀌지 않을때
		if( lowcode[ch] <= highcode[ch]) {
			midcode[ch] = (lowcode[ch] + highcode[ch]) / 2;
			if (MIDMAX >= volt && MIDMIN <= volt) { //탐색 성공
				// do nothing
				midcode[ch] = premidcode[ch];
				lowcode[ch] = 0x00;   // restart
				highcode[ch] = 0x7F;

				return midcode[ch];
			}
			else if (volt < MIDMIN) {        // 신호가 낮아서 ATTN level을 낮춘다.
				highcode[ch] = midcode[ch] - 1;
				midcode[ch] = lowcode[ch];
				return midcode[ch];
			}
			else if (volt > MIDMAX) {        // 신호가 높아서, ATTN level을 높힌다.
				lowcode[ch] = midcode[ch] + 1;
				midcode[ch] = highcode[ch];
				return midcode[ch];
			}
		} else {  // 뒤바뀔때, 재검색가능 으로 설정
			if( MIDMAX < volt )
			{
				lowcode[ch] = 0x00;
				highcode[ch] = 0x7F;
			} else if ( MIDMIN > volt  )
			{
     				lowcode[ch] = 0x00;
     				highcode[ch] = 0x7F;
     				//midcode[ch] = 0x00;
			}
		}

	} // end of 범위밖

	return midcode[ch];
}

// calPowerRSSI(midcode[curr->ch], maxSensVal[curr->ch]);

float calPowerRSSI(uint8_t attncode, int adccode )
{
	float powerlevel = 0.0;
	float adcvalue = 0.0;
	float attnlevel = attncode/4.0;


	adcvalue = (adccode/65535.0)*3.3;
	powerlevel = level2Power(adcvalue)+attnlevel;


	return (powerlevel-55.0); // -55 for 4 LNA gain + 5 for other loss


	// return attnlevel;

	// return (float)(adccode/100.0);

	// return adcvalue;

}




unsigned int hcf(unsigned int n1, unsigned int n2) {
	if (n2 != 0)
		return hcf(n2, n1 % n2);
	else
		return n1;
}

void valTestInit() {
	for (U8 dd = 0; dd < REG_SIZE; dd++)
		valInAddr[dd] = 0x00;

	for(U8 dd=0; dd<MAXCH; dd++)
	{
		currAttnVal[dd]=0;
		maxSensVal[dd]=0;
		highcode[dd] = 0x7E;
		lowcode[dd] = 0x00;
		midcode[dd]=0x00;
	}

}

uint64_t Euclidean(uint64_t a, uint64_t b)
{
	if( b ==0 )
		return a;
	else
		return Euclidean(b, a%b);
}

/* 
 int main(void)
 {
 uint64_t a = Euclidean(11, 10);
 printf("%d \n", a);
 return 0;
 }
 */

//////// CRC calculation
U16 crc16_cal(U8 *a, U8 length) {
	int i;
	U16 crc = 0xffff;
	for (U8 j = 0; j < length; j++) {

		crc ^= (uint16_t) a[j];
		for (i = 0; i < 8; ++i) {
			if (crc & 1)
				crc = (crc >> 1) ^ 0xA001;
			else
				crc = (crc >> 1);
		}
	}

	return crc;
}

// char val[] size is 3; 0x000000, fracdigit 0 or 2
float hexatofloat(U8 *val, int fractdigit) {
	float k=0.0;
	if (fractdigit == 2) {
		k = (val[0] >> 4 & 0x0F) * 1000 + (val[0] & 0x0F) * 100
			+ (val[1] >> 4 & 0x0F) * 10 + (val[1] & 0x0F) * 1
			+ (val[2] >> 4 & 0x0F) / 10.0 + (val[2] & 0x0F) / 100.0;
	} else if (fractdigit == 0) {
		k = (val[0] >> 4 & 0x0F) * 100000 + (val[0] & 0x0F) * 10000
			+ (val[1] >> 4 & 0x0F) * 1000 + (val[1] & 0x0F) * 100
			+ (val[2] >> 4 & 0x0F) * 10.0 + (val[2] & 0x0F) * 1.0;
	}
	return k;
}

unsigned char crc8(unsigned char *data, unsigned char length) {
	unsigned char count;
	unsigned int sum = 0;

	for (count = 0; count < length; count++)
		sum = (sum + data[count]) & 0xFF;

	return sum;
}

// to get reverse float order from float to U8

void comm_rs485_rvsFloat_order(U8 *dest, float *val) {
	U8 fbuff[4];

	//// store float to temperary buffer to reversing
	memcpy(fbuff, val, 4);

	//// reversing store to register

	memcpy(dest + 3, fbuff + 0, 1);
	memcpy(dest + 2, fbuff + 1, 1);
	memcpy(dest + 1, fbuff + 2, 1);
	memcpy(dest + 0, fbuff + 3, 1);

}
// to get reverse float order from U8 to float value
void comm_rs485_rcvFloat_order(void *dest, U8 *src) {
	U8 fbuff[4];

	//// store float to temperary buffer to reversing
	memcpy(fbuff, src, 4);

	//// reversing store to register
	memcpy(dest + 3, fbuff + 0, 1);
	memcpy(dest + 2, fbuff + 1, 1);
	memcpy(dest + 1, fbuff + 2, 1);
	// RF off HMC832 //
	// RF OFF
	for (int j = 2; j < 3; j++) {
		HMC832_writeReg(onoffregs[j], onoffregs[j]);
		//DWT_Delay_us(200);
	}
	memcpy(dest + 0, fbuff + 3, 1);

}

// RTX_V1 PC13,=> PC2
// RTX_V2 PC14 => PC3
// able 4. SKY13446-374LF Truth Table 1
//         V1 (Pin 5) V2 (Pin 2)    ANT to RX Path  //  ANT to TX Path
//             1          0          Insertion loss / Isolation      ( RX on )
//             0          1          Isolation      / Insertion loss ( TX 0n )
#define PWR_PORT GPIOC
#define PWRCTL0_PIN GPIO_PIN_4
#define PWRCTL1_PIN GPIO_PIN_5 // not used 2021.08.18

#define PWR_ON() HAL_GPIO_WritePin(PWR_PORT, PWRCTL0_PIN, GPIO_PIN_SET); \
HAL_GPIO_WritePin(PWR_PORT, PWRCTL1_PIN, GPIO_PIN_SET)

#define PWR_OFF() HAL_GPIO_WritePin(PWR_PORT, PWRCTL0_PIN, GPIO_PIN_RESET); \
HAL_GPIO_WritePin(PWR_PORT, PWRCTL1_PIN, GPIO_PIN_RESET)


#define SKYSW_TX_PIN GPIO_PIN_0 // looks changed PIN2 and PIN3
#define SKYSW_RX_PIN GPIO_PIN_2

#define SKYSW_TX_PORT GPIOC
#define SKYSW_RX_PORT GPIOC

/*
*/

//Define the pins tp connect
#define SKYSW_TX_ON() HAL_GPIO_WritePin(SKYSW_TX_PORT, SKYSW_TX_PIN, GPIO_PIN_RESET);

#define SKYSW_RX_ON() HAL_GPIO_WritePin(SKYSW_TX_PORT, SKYSW_TX_PIN, GPIO_PIN_SET);

#define SKYSW_RTX_OFF() HAL_GPIO_WritePin(SKYSW_TX_PORT, SKYSW_TX_PIN, GPIO_PIN_SET);


// PB12: TX_SW
// PB13: TX_SW1
// PB14: RX_SW
#define F2910_TX_PIN GPIO_PIN_12
#define F2910_TX1_PIN GPIO_PIN_12
#define F2910_RX_PIN GPIO_PIN_13

#define F2910_RTX_PORT GPIOB

#define F2910_TX_ON() HAL_GPIO_WritePin(F2910_RTX_PORT, F2910_RX_PIN, GPIO_PIN_RESET); \
HAL_GPIO_WritePin(F2910_RTX_PORT, F2910_TX1_PIN, GPIO_PIN_SET); \
HAL_GPIO_WritePin(F2910_RTX_PORT, F2910_TX_PIN, GPIO_PIN_SET)

/*
*/

#define F2910_RX_ON() HAL_GPIO_WritePin(F2910_RTX_PORT, F2910_TX_PIN, GPIO_PIN_RESET); \
HAL_GPIO_WritePin(F2910_RTX_PORT, F2910_TX1_PIN, GPIO_PIN_RESET); \
nsdelay(60); \
HAL_GPIO_WritePin(F2910_RTX_PORT, F2910_RX_PIN, GPIO_PIN_SET)

// delay(52)

#define F2910_RTX_OFF() HAL_GPIO_WritePin(F2910_RTX_PORT, F2910_TX_PIN, GPIO_PIN_RESET); \
HAL_GPIO_WritePin(F2910_RTX_PORT, F2910_TX1_PIN, GPIO_PIN_RESET); \
HAL_GPIO_WritePin(F2910_RTX_PORT, F2910_RX_PIN, GPIO_PIN_RESET)

/*
*/

/// LTC5582 power sensor enable
#define DET_EN_PORT GPIOD
#define DET_EN_PIN GPIO_PIN_8

#define DET_ENABLE() HAL_GPIO_WritePin(DET_EN_PORT, DET_EN_PIN, GPIO_PIN_SET);
#define DET_DISABLE() HAL_GPIO_WritePin(DET_EN_PORT, DET_EN_PIN, GPIO_PIN_RESET);

///// RS422 check pins
/*
#define RTSIN_PORT GPIOA
#define RTSIN_PIN GPIO_PIN_9
*/


///// RS422 control pins
/*
#define RTSCTL_PORT GPIOC
#define RTSCTL_PIN1 GPIO_PIN_6
#define RTSCTL_PIN2 GPIO_PIN_7
*/

#define CTL_PIN1 GPIO_PIN_8
#define CTL_PIN2 GPIO_PIN_9
#define CTLIN_PORT GPIOD
#define CTLIN_PIN GPIO_PIN_15


///// RS422 TX enable
#define TXENABLE_PORT GPIOA
#define TXENABLE_PIN GPIO_PIN_8


void ch1_on()
{
	  HAL_GPIO_WritePin(GPIOD, LED_CH1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, LED_CH2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOC, LED_CH3_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, LED_CH4_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOC, LED_CH5_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, LED_CH6_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, BIT_EN_Pin, GPIO_PIN_RESET);

	  /*
	  HAL_GPIO_WritePin(GPIOD, FC1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, FC2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB, FC3_Pin, GPIO_PIN_SET);
	  */
	    // FC1=H, FC2=H, FC3=H
	    GPIOD->BSRR = (1U << 6) | (1U << (14));   // PD6, PD14 High
	    GPIOB->BSRR = (1U << 3);                // PB3 High

}

void ch2_on()
{
	  HAL_GPIO_WritePin(GPIOD, LED_CH1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, LED_CH2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, LED_CH3_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, LED_CH4_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOC, LED_CH5_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, LED_CH6_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, BIT_EN_Pin, GPIO_PIN_RESET);
	  /*
	  HAL_GPIO_WritePin(GPIOD, FC1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOD, FC2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB, FC3_Pin, GPIO_PIN_RESET);
	  */
	    // FC1=L, FC2=H, FC3=L
	    GPIOD->BSRR = (1U << (6 + 16)) | (1U << (14));  // PD6 Low, PD14 High
	    GPIOB->BSRR = (1U << (3 + 16));               // PB3 Low
}
void ch3_on()
{
	  HAL_GPIO_WritePin(GPIOD, LED_CH1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, LED_CH2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOC, LED_CH3_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOD, LED_CH4_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOC, LED_CH5_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, LED_CH6_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, BIT_EN_Pin, GPIO_PIN_RESET);
	  /*
	  HAL_GPIO_WritePin(GPIOD, FC1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOD, FC2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB, FC3_Pin, GPIO_PIN_SET);
	  */
	    // FC1=L, FC2=H, FC3=H
	    GPIOD->BSRR = (1U << (6 + 16)) | (1U << (14));  // PD6 Low, PD14 High
	    GPIOB->BSRR = (1U << 3);
}
void ch4_on()
{
	  HAL_GPIO_WritePin(GPIOD, LED_CH1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, LED_CH2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOC, LED_CH3_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, LED_CH4_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, LED_CH5_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, LED_CH6_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, BIT_EN_Pin, GPIO_PIN_RESET);

	  /*
	  HAL_GPIO_WritePin(GPIOD, FC1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, FC2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, FC3_Pin, GPIO_PIN_RESET);
	  */
	    // FC1=H, FC2=L, FC3=L
	    GPIOD->BSRR = (1U << 6) | (1U << (14+16));  // PD6 High, PD14 Low
	    GPIOB->BSRR = (1U << (3 + 16));               // PB3 Low
}

void ch5_on()
{
	  HAL_GPIO_WritePin(GPIOD, LED_CH1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, LED_CH2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOC, LED_CH3_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, LED_CH4_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOC, LED_CH5_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOD, LED_CH6_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, BIT_EN_Pin, GPIO_PIN_RESET);

	  /*
	  HAL_GPIO_WritePin(GPIOD, FC1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, FC2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, FC3_Pin, GPIO_PIN_SET);
	  */
	    // FC1=H, FC2=L, FC3=H
	    GPIOD->BSRR = (1U << 6) | (1U << (14+16));  // PD6 High, PD14 Low
	    GPIOB->BSRR = (1U << 3);                      // PB3 High
}

void ch6_on()
{
	  HAL_GPIO_WritePin(GPIOD, LED_CH1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, LED_CH2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOC, LED_CH3_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, LED_CH4_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOC, LED_CH5_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, LED_CH6_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOD, BIT_EN_Pin, GPIO_PIN_RESET);

	  /*
	  HAL_GPIO_WritePin(GPIOD, FC1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, FC2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB, FC3_Pin, GPIO_PIN_RESET);
	  */
	    // FC1=H, FC2=H, FC3=L
	    GPIOD->BSRR = (1U << 6) | (1U << (14));         // PD6, PD14 High
	    GPIOB->BSRR = (1U << (3 + 16));               // PB3 Low
}
void rxon_and_alloff()
{
	  HAL_GPIO_WritePin(GPIOD, LED_CH1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, LED_CH2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOC, LED_CH3_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, LED_CH4_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOC, LED_CH5_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, LED_CH6_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, BIT_EN_Pin, GPIO_PIN_RESET);

	  /*
	  HAL_GPIO_WritePin(GPIOD, FC1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOD, FC2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, FC3_Pin, GPIO_PIN_RESET);
	  */
	    // FC1=L, FC2=L, FC3=L
	    GPIOD->BSRR = (1U << (6 + 16)) | (1U << (14+16));  // PD6, PD14 Low
	    GPIOB->BSRR = (1U << (3 + 16));                      // PB3 Low
}

void bit_on()
{
	  HAL_GPIO_WritePin(GPIOD, LED_CH1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, LED_CH2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, LED_CH3_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOD, LED_CH4_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, LED_CH5_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOD, LED_CH6_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOD, BIT_EN_Pin, GPIO_PIN_SET);

	  /*
	  HAL_GPIO_WritePin(GPIOD, FC1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOD, FC2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, FC3_Pin, GPIO_PIN_RESET);
	  */
	    // FC1=L, FC2=L, FC3=L, BIT_EN=H
	    GPIOD->BSRR = (1U << (6 + 16)) | (1U << (14+16));  // PD6, PD14 Low
	    GPIOB->BSRR = (1U << (3 + 16));                      // PB3 Low
	    GPIOB->BSRR = (1U << BIT_EN_Pin);                    // BIT_EN High
}

volatile uint32_t awd_ticks = 0;
volatile uint8_t awd_flag = 0;

void ch_timing_measure(int ch)
{

    //printf("Hello World");
	 //ADC1_AWD_Config_Code(3.3f, 1.6f);
    //printf("Hello World");
	 HAL_Delay(200);
    awd_flag = 0;
    DWT->CYCCNT = 0;     // 기준을 0으로 리셋

    if(ch == 1)
    {
    	ch1_on();
    }else if(ch == 2)
    {
    	ch2_on();
    }else if(ch == 3)
    {
    	ch3_on();
    }else if(ch == 4)
    {
    	ch4_on();
    }else if(ch == 5)
    {
    	ch5_on();
    }else if(ch == 6)
    {
    	ch6_on();
    }

    // AWD가 트리거 될 때까지 대기
    // printf("Hello World");
    // while (!awd_flag);
    //printf("AWD configured on ADC1_IN16, High=%u (%.2f V)\r\n", 0.0, 0.0);

    float delay_us = (awd_ticks / 400.0f);  // 400MHz = 2.5ns per tick
    // printf("[CH6] Delay = %.3f us\r\n", delay_us);
    peakFreq = delay_us;
}

int waitExecCtl(int logicSel )
{
	int ctlChecker = 0;
	int w=0;
	if( logicSel )
	{
		w=0;
		while(!HAL_GPIO_ReadPin(CTLIN_PORT, CTLIN_PIN))
		{
			if( w > 60000 || (blockCtrl == NO_MEAS_BLOCK)) // for 60 secs
			{
				// pause block
				ctlChecker = 0;
				return ctlChecker;
			}
			if(firstLoop)
			{
				DWT_Delay_us(1);
			}
			// iteration
			w++;
		}
	} else {
		w=0;
		while(HAL_GPIO_ReadPin(CTLIN_PORT, CTLIN_PIN))
		{
			if( w > 60000 || (blockCtrl == NO_MEAS_BLOCK)) // for 60 secs
			{
				// pause block
				ctlChecker = 0;
				return ctlChecker;
			}
			if(firstLoop)
			{
				DWT_Delay_us(1);
			}
			// iteration
			w++;
		}

	}

	ctlChecker = 1;
	return ctlChecker;
}




int checkExecCtl()
{
	int ctlChecker = 0;
	if(HAL_GPIO_ReadPin(CTLIN_PORT, CTLIN_PIN))
	{
		ctlChecker = 0;
		return ctlChecker;
	} else {
		ctlChecker = 1;
		return ctlChecker;
	}
}


void setExecCtl(int id, int onoff)
{
	/*
	if(id == 1)
	{
		if(onoff == 1)
			HAL_GPIO_WritePin(RTSCTL_PORT, CTL_PIN1, GPIO_PIN_SET);
		if(onoff == 0)
			HAL_GPIO_WritePin(RTSCTL_PORT, CTL_PIN1, GPIO_PIN_RESET);
	} else if( id == 2 )
	{
		if(onoff == 1)
			HAL_GPIO_WritePin(RTSCTL_PORT, CTL_PIN2, GPIO_PIN_SET);
		if(onoff == 0)
			HAL_GPIO_WritePin(RTSCTL_PORT, CTL_PIN2, GPIO_PIN_RESET);
	} else {
		if(onoff == 1)
		{
			HAL_GPIO_WritePin(RTSCTL_PORT, CTL_PIN1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RTSCTL_PORT, CTL_PIN2, GPIO_PIN_SET);

		}
		if(onoff == 0)
		{
			HAL_GPIO_WritePin(RTSCTL_PORT, CTL_PIN1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RTSCTL_PORT, CTL_PIN2, GPIO_PIN_RESET);
		}
	}
	*/

}

void rtsLock(int id, int onoff) // 0 for unlock, 1 for lock
{
	/*
	if(id == 1)
	{
		if(onoff == 1)
			HAL_GPIO_WritePin(RTSCTL_PORT, RTSCTL_PIN1, GPIO_PIN_RESET);
		if(onoff == 0)
			HAL_GPIO_WritePin(RTSCTL_PORT, RTSCTL_PIN1, GPIO_PIN_SET);
	} else if( id == 2 )
	{
		if(onoff == 1)
			HAL_GPIO_WritePin(RTSCTL_PORT, RTSCTL_PIN2, GPIO_PIN_RESET);
		if(onoff == 0)
			HAL_GPIO_WritePin(RTSCTL_PORT, RTSCTL_PIN2, GPIO_PIN_SET);
	} else {
		if(onoff == 1)
		{
			HAL_GPIO_WritePin(RTSCTL_PORT, RTSCTL_PIN1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RTSCTL_PORT, RTSCTL_PIN2, GPIO_PIN_RESET);

		}
		if(onoff == 0)
		{
			HAL_GPIO_WritePin(RTSCTL_PORT, RTSCTL_PIN1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RTSCTL_PORT, RTSCTL_PIN2, GPIO_PIN_SET);
		}

	}
	*/
}


//#include"stm32_eeprom_func.h"
void nsdelay(uint16_t delay) {
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	while (__HAL_TIM_GET_COUNTER(&htim3) < delay)
		;
}

float floatmovingAvg(float *ptrArrNumbers, double *ptrSum, int pos, int len,
		float nextNum) {
	//Subtract the oldest number from the prev sum, add the new number
	*ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
	//Assign the nextNum to the position in the array
	ptrArrNumbers[pos] = nextNum;
	//return the average
	return *ptrSum / len;
}

int movingAvg(int *ptrArrNumbers, long *ptrSum, int pos, int len, int nextNum) {
	//Subtract the oldest number from the prev sum, add the new number
	*ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
	//Assign the nextNum to the position in the array
	ptrArrNumbers[pos] = nextNum;
	//return the average
	return *ptrSum / len;
}




// handling size, option for 2 bytes handling or 3 bytes handline
// if last 1, it is last data for request
// if last 0, it is not the last
// float value to host anaValFltd[] to host
U8 x68_massTxFrameData(int idx, U8 size, U8 option, U8 last) {

	//S32 code=0; // for max AABBCC 3 bits
	char tmpbuf[NUM_SIZE8 + 1];

	bufIndex = 4; // 0:x68, 1:Len, 2:id, 3 fncode
	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,	DAC_ALIGN_12B_R, maxavgval);
	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,	DAC_ALIGN_12B_R, 65535);



	TxData[0] = 0x68;
	TxData[2] = myID;
	TxData[3] = FnCode | 0x80;


	/////////////////////////////////////////
	// body header creation, indicator to show the last one
	// or not + data counts in bytes

	/////////////////////////
	// stamp if it is the last rest data group or not, if the rest 1x
	snprintf(tmpbuf, 6, "%05d", idx);

	if (last == 0)
		TxData[bufIndex++] = 0x00 | (tmpbuf[0] - 0x30);
	else
		TxData[bufIndex++] = 0x10 | (tmpbuf[0] - 0x30);

	if (option == 3)
		TxData[bufIndex++] = (tmpbuf[1] - 0x30) << 4 | (tmpbuf[2] - 0x30);

	TxData[bufIndex++] = (tmpbuf[3] - 0x30) << 4 | (tmpbuf[4] - 0x30);

	///////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////

	/////////////////////////////////////////
	// body creation
	// loop from start index to start index + size
	for (int i = idx; i < (idx + size); i++) {

		////// display on OLED /////////////////
		// for +/- selection, ch1, sending anaVal to host computer
		snprintf(tmpbuf, 9, "%+07.02f", (float)(anaValFltd[i]*1.0));
		if (option == 3) {
			if (anaVal[i] < 0)
				TxData[bufIndex++] = 0x10 | (tmpbuf[1] - 0x30); // XA
			else
				TxData[bufIndex++] = (tmpbuf[1] - 0x30); // XA
			TxData[bufIndex++] = (tmpbuf[2] - 0x30) << 4 | (tmpbuf[3] - 0x30); //BB
			TxData[bufIndex++] = (tmpbuf[5] - 0x30) << 4 | (tmpbuf[6] - 0x30); // .CC
		} else if (option == 2) {
			if (anaVal[i] < 0)
				TxData[bufIndex++] = 0x10 | (tmpbuf[1] - 0x30); // XA
			else
				TxData[bufIndex++] = (tmpbuf[1] - 0x30); // XA
			TxData[bufIndex++] = (tmpbuf[2] - 0x30) << 4 | (tmpbuf[3] - 0x30); //BB
		}

	} // end of loop

	TxLen = bufIndex;
	TxData[1] = TxLen;
	TxData[bufIndex] = crc8(TxData + 1, TxLen - 1);
	return 0;
}

// transfer


// temperary txprocess for test
void txProcess(uint8_t *ptrTxdata, uint8_t len) {
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);

	HAL_Delay(10);
	HAL_UART_Transmit(&huart1, ptrTxdata, len + 1, 100);
	// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

}

/// for RS485 STANDARD CRC procedure //////////////////
#include"std_rs485_func.h"

U8 x68_rxCrcCheck8(U8 len, int idx) {
	U8 crcRslt = 0xFF;

	crcRslt = crc8(RxData +idx+1, len-1);			//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,	DAC_ALIGN_12B_R, maxavgval);
	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,	DAC_ALIGN_12B_R, 65535);

	// if (crcRslt == RxData[RxSize - 2]) { // only for id#1
	if (crcRslt == RxData[idx+len]) {
		return crcRslt;
	} else
		return 0;
}



void x68_massTxProcess() {
	/*
	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	   HAL_Delay(10);
*/
	HAL_UART_Transmit(&huart1, (uint8_t *) (&TxData), TxLen + 1, 100);

	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

}

void x68_txProcess() {

	/* blocking debug check
   	   HAL_GPIO_WritePin(RTSCTL_PORT, RTSCTL_PIN1, GPIO_PIN_RESET);
   	   DWT_Delay_us(100);*/

	// NAND gate check if all pin is HIGH ( unlock = idle )

	int txChecker = 0;
	if(myID > 0)
	{
		do
		{
			txChecker = 0;
			for(int i=0; 10000>i; i++)
			{
				DWT_Delay_us(1);
				/*
				if(HAL_GPIO_ReadPin(RTSIN_PORT, RTSIN_PIN))
				{
					txChecker = 1;
				} else {
					txChecker = 0;
				}
				*/
				txChecker = 0;
			}
		} while ( txChecker == 1 );
	}


	if(myID > 0)
	{
		rtsLock(myID, 1); // lock for myID xfer
		//HAL_Delay(3);
	}

	HAL_GPIO_WritePin(TXENABLE_PORT, TXENABLE_PIN, GPIO_PIN_SET);
	HAL_Delay(3);
	//DWT_Delay_us(100);
	HAL_UART_Transmit(&huart1, (uint8_t *) (&TxData), TxLen + 1, 100);

	HAL_GPIO_WritePin(TXENABLE_PORT, TXENABLE_PIN, GPIO_PIN_RESET);

	if(myID > 0)
	{
		rtsLock(myID, 0); // unlock for other ID xfer
	}

}

//////////////////////////////////////////////////////////////////
// sending data format
// 0x68, 0xLN, 0xID, 0xFn(88),
// 0xCH 0xAABBCC

void x68_txFreq(U8 ch, float val) {
	TxData[0] = 0x68;
	TxData[1] = 0x08;
	TxData[2] = myID;
	TxData[3] = FnCode | 0x80;
	TxData[4] =  ch;
	char freqbuf[9];

	snprintf(freqbuf, 8, "%+07.02f", val);

	TxData[5] = (freqbuf[1] - 0x30); // XA
	TxData[6] = (freqbuf[2] - 0x30) << 4 | (freqbuf[3] - 0x30); //BB
	TxData[7] = (freqbuf[5] - 0x30) << 4 | (freqbuf[6] - 0x30); // .CC

	TxData[8] = crc8(TxData + 1, 0x08 - 1);
}			//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,	DAC_ALIGN_12B_R, maxavgval);
//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,	DAC_ALIGN_12B_R, 65535);

//////////////////////////////////////////////////////////////////
// sending data format
// 0x68, 0xLN, 0xID, 0xFn(88),
// 0xCH 0xAABBCC(freq) 0xAABBCC(max power)


void x68_txFreqNpwr(U8 ch, float val, float pwr) {
	TxData[0] = 0x68;
	TxData[1] = 0x0B;
	TxData[2] = myID;
	TxData[3] = FnCode | 0x80;
	TxData[4] =  ch;
	char freqbuf[9], pwrbuf[9];

	snprintf(freqbuf, 8, "%+07.02f", val);

	TxData[5] = (freqbuf[1] - 0x30); // XA
	TxData[6] = (freqbuf[2] - 0x30) << 4 | (freqbuf[3] - 0x30); //BB
	TxData[7] = (freqbuf[5] - 0x30) << 4 | (freqbuf[6] - 0x30); // .CC

	snprintf(pwrbuf, 8, "%+07.02f", pwr);
	if( pwr < 0.0 && pwr > -100.0)
	{
		pwr = pwr * -1.0; // making plus
		pwr = pwr+100;
		snprintf(pwrbuf, 8, "%+07.02f", pwr);
	}
	TxData[8] = (pwrbuf[1] - 0x30); // XA
	TxData[9] = (pwrbuf[2] - 0x30) << 4 | (pwrbuf[3] - 0x30); //BB
	TxData[10] = (pwrbuf[5] - 0x30) << 4 | (pwrbuf[6] - 0x30); // .CC


	TxData[11] = crc8(TxData + 1, 0x0B - 1);
}


void x68_txVal(U8 val) {
	TxData[0] = 0x68;
	TxData[1] = 0x05;
	TxData[2] = myID;
	TxData[3] = FnCode | 0x80;
	TxData[4] =  val; // for dummy data or id for future use
	TxData[5] = crc8(TxData + 1, 0x05 - 1);
}


void x68_txEcho(U8 val) {
	TxData[0] = 0x68;
	TxData[1] = 0x05;
	TxData[2] = myID;
	TxData[3] = FnCode | 0x80;
	TxData[4] =  val; // for dummy data or id for future use
	TxData[5] = crc8(TxData + 1, 0x05 - 1);
}




void x68_txData(U8 val) {
	TxData[0] = 0x68;
	TxData[1] = 0x05;
	TxData[2] = myID;
	TxData[3] = FnCode | 0x80;
	TxData[4] =  val; // for dummy data or id for future use
	TxData[5] = crc8(TxData + 1, 0x05 - 1);
}


///////////////////////////////////// RF Sensing Process //////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
/*
void HMC832_Init(void)
{

    HAL_Delay(5);                      // 안정 대기
}
*/

// 외부 레퍼런스 클럭 (예: 16 MHz 크리스탈)
#define FREF_HZ 16000000.0
#define R_DIV    1           // R Divider 값 (기본: 1)


// ──────────────── Divider 매핑 ────────────────
typedef struct { uint32_t div; uint8_t code; } divmap_t;
static const divmap_t kDivMap[] = {
    {1,0},{2,1},{4,2},{8,3},{16,4},{32,5},{64,6},{128,7}
};

// Output Divider 매핑
// ----------------------

// ----------------------
// 주파수 설정 함수
// ----------------------


// ──────────────── Divider 세팅 ────────────────

// ──────────────── Lock Detect (SPI Read Reg0x07) ────────────────

// -------------------------------------
// Divider 인코딩 매핑 함수
// -------------------------------------



// ──────────────── 원하는 출력 주파수 설정 ────────────────
/*
*/
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

    unsigned char outdivider, rdivider=1, kval;
    unsigned int reg02, reg03, reg04, prereg03;
    unsigned int fpd, fxtal, nFRACT, nINT;
    float nTOT;
    U8 anaCal;
    U8 *anaCalPtr= &anaCal;
    reg02 = rdivider;
    kval = 0x01;

    float d_min_ideal = VCO_MIN_FREQ_MHZ / (fout/1e6);

    /*https://ko.xhamster.desi/videos/gently-lovely-caressing-my-first-solo-video-xhHUNBs
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
            if ((float)current_D >= d_min_ideal) {
                kval = current_D;
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




    // printf("write 0x%02x, 0x%06x,  \n\n", 0x03, reg03);
    // printf("write 0x%02x, 0x%06x,  \n\n", 0x04, reg04);

	for (int j = 0; j < 3; j++) {
		HMC832_writeReg(setFractRegs[j], setFractVals[j]);
    	HAL_Delay(us);
	}
	HMC832_vspiWriteReg(0x05, 0x02, kval);
    HAL_Delay(us);
}





void HMC832_SetExactFrequency(double fout, double us)
{

    unsigned char ndivider, outdivider, rdivider=0x01, kval;
    unsigned int reg0C, reg03, reg04, prereg03;
    float fpd, fxtal, fNcurr;
    U8 anaCal;
    U8 *anaCalPtr= &anaCal;
    float d_min_ideal = VCO_MIN_FREQ_MHZ / (fout/1e6);

    /*
	if (fout/1e6 < 1500.0) {
		for (int i = 62; i >= 2; i -= 2) {
			if ((fout/1e6 * i) < 3000.0) {
				ndivider = i;
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
            if ((float)current_D >= d_min_ideal) {
                kval = current_D;
                ndivider = current_D;
                break;
            }
        }
    }


	outdivider = kval * rdivider; //2x8=16
	fxtal = 16e6;
	fpd = fxtal / rdivider; // 2,000,000
	// fgcdmin = fpd / pow(2, 14);

    reg03 = floor(fout * kval / fpd);
	reg0C = fpd / (hcf(fpd, fout * kval));
	fNcurr = floor(fout * kval / fpd) * fpd;

	setvals[0] = reg03;


	// exact freq에서는 reg04는 setup하지 않는다..
	//reg04 = (pow(2, 24) * (fout * kval - fNcurr) / fpd);
	setvals[1] = reg0C;

    // printf("write 0x%02x, 0x%06x,  \n\n", 0x03, reg03);
    // printf("write 0x%02x, 0x%06x,  \n\n", 0x04, reg04);

	//setvals[1] = reg0C;
	// set HMC832 frequency register 03//
	// initialize HMC832 //



	for (int j = 0; j < 2; j++) {
		HMC832_writeReg(setregs[j], setvals[j]);
    	HAL_Delay(us);
	}
	HMC832_vspiWriteReg(0x05, 0x02, kval);
    HAL_Delay(us);

}

// 고정 주파수 테스트
void rfSensing( U8 ch, float stfrequency, float spfrequency)
{

	peakFreq = 0.0;
	peakVal = 0;
	/// get max frequency
	maxfreqval = 0;
	maxfreqidx = 0;
	uint32_t tempEdge=0;
	// uint32_t Rdivider = 2000;

	// mass data handling
	valIndex = 0;
	// end of mass datahandling


	// atten level adjust
	maxSensVal[ch] = 0;


	// initialize variables //
	for(int i=0; MAXINDX>i; i++)
	{
		anaValTot[i]=0;
	}

	if (spfrequency - stfrequency > 5) {
		tempEdge = 16;
	} else {
		tempEdge = 6;
	}


	////////////////////// Display /////////////////////////////

	///////////////////// End of Display ////////////////////////

	// /////////////////////////////////////////////////////////
	// get value from RX data frame

	freqstep = (float) ((spfrequency - stfrequency) / division);

	stfreqM = (stfrequency * 1000000);
	spfreqM = (spfrequency * 1000000);
	fstepM = ((spfreqM - stfreqM) / division);

	if (freqstep < 0) {
		return;
	}

	if (spfrequency < 1500.0) {
		for (int i = 62; i >= 2; i -= 2) {
			if ((spfrequency * i) < 3000.0) {
				ndivider = i;
				break;
			}
		}
	}

	SKYSW_RTX_OFF();
	F2910_RTX_OFF();

   rxon_and_alloff();

	/////////////////////////// End of RTX H/W Control ////////////////////////////////

	// initialize

	for (int i = 0; i < 45; i++) {
		HMC832_writeReg(initregs[i], initvals[i]);
		HAL_Delay(10);
		//DWT_Delay_us(200);
		__NOP();
	}
	initvals[21] = initauto;


	F2910_TX_ON();
	SKYSW_TX_ON();

	DWT_Delay_us(1);

	HMC832_SetFractFrequency1((stfreqM+spfreqM)/2, 1);

	x68_txFreq(1, 111.11);
	HAL_Delay(10);
	x68_txProcess();
	HAL_Delay(10);

	if(ch == 1) {
    	ch_timing_measure(1);
	} else if (ch == 2) {
		ch_timing_measure(2);
	} else if ( ch == 3) {
		ch_timing_measure(3);
	} else if ( ch == 4 ) {
		ch_timing_measure(4);
	} else if (ch == 5 ) {
		ch_timing_measure(5);
	} else if ( ch == 6 ) {
		ch_timing_measure(6);
	} else {
		bit_on();
	}
	HAL_Delay(5000);
	rxon_and_alloff();

	//////////////////////////////// MAXLOOP Loop start /////////////////////////////
	/*

		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4095);
		DWT_Delay_us(1);

				HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buff, MAXAVG);
				//


				DWT_Delay_us(1);

				//HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buff, MAXAVG);
				F2910_RX_ON();
				SKYSW_RX_ON();

				while (ADC_ConvCpltFlag == 0)
					;

				ADC_ConvCpltFlag = 0;
				HAL_ADC_Stop_DMA(&hadc1);

				for (tmp = 0; tmp < MAXAVG; tmp++) {
					ADC_monitor = *((uint16_t*) (adc_buff + tmp));
					sum += ADC_monitor;
					// get max value
					if (ADC_monitor > maxval) {
						maxval = ADC_monitor;


					}
					if (ADC_monitor < minval && ADC_monitor > 0) {
						minval = ADC_monitor;
						minidx = tmp;
					}

				}

	*/



	SKYSW_RTX_OFF();
	F2910_RTX_OFF();
	PWR_OFF();

	//DET_DISABLE();

	// initialize fraction for next integer set
	setvals[0] = 0x000000;
	setvals[1] = 0x000000;
	setvals[2] = 0x000000;

	pullstart=0;

	/////////////////// send freq through
	x68_txFreq(ch, peakFreq);
	// x68_txFreqNpwr(ch, peakFreq, calPowerRSSI(midcode[ch], maxSensVal[ch]));
	x68_txProcess();

	/////////////////// END Of freq sending
	if (MAXINDX < valIndex)
		valIndex = MAXINDX - 1;

	//HAL_Delay(10);
	//delay_ms(10);

	//blockCtrl = MASS_BLOCK;
	vergine = 0;
	preCh = ch;
}


///////////////////////////////// End of RF Sensing Process ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////// 68 protocol Rx Process //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
U8 x68_rxProcess() {

	RxDLen = RxData[1];
	RxAddr = RxData[2];
	FnCode = RxData[3];

	if (myID == RxAddr || 0x00 == RxAddr) {

		uartit_on=1;

		/////////////////////////////////////////////////////
		// RF TX on, off, sweep on, sweep off FnCode == 0x01
		/////////////////////////////////////////////////////
		if (FnCode == 0x01) {
			TxLen = 0x05;
			blockCtrl = CA_BLOCK;

			//////////// echo back /////////////// programmed
			if (RxData[6] == 0x00) {

				uartit_on=1;
				FnCode=0x01;
				TxLen = 0x05;
				x68_txEcho(0x00);
				x68_txProcess();

			}

		} //// end of FnCode == 0x01


		/////////////////////////////////////////////////////////////
		//// Fn0x09 multiple channel test start
		//// Fn08: max power level sending also with multi channel test
		/////////////////////////////////////////////////////////////
		// 0x68, 0xLN, 0xID, 0xFn(08 | 09),  0x000000(division)
		// 0xCH, 0x000000 (st), 0x000000(sp)
		// 0xCH, 0x000000 (st), 0x000000(sp) repeat
		// receiving data format


		else if (FnCode == 0x09 || FnCode == 0x08 ) {

			TxLen = 0x05;
			int RxLen = RxData[1];
			int chanSize = RxLen/7.0-1;

			blockCtrl = NO_MEAS_BLOCK;

			if(FnCode == 0x08)
			{
				powerMeasOn = 1;
			} else {
				powerMeasOn = 0;
			}

			// display calibration
			uartit_on=1;

			// HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET); // start cal display


			if( chanSize > 0)
			{



		    		//// SPI initialize for HMC832 ////////////////
	     			// SCLK to low
	     			pullstart=0;
	     			MX_SPI1_Init();
	     			//HAL_SPI_MspDeInit(&hspi1);
	     			HAL_SPI_MspInit(&hspi1);

	     			// SEN to low
	     			HMC832_SEN_Clr();

	     			HAL_Delay(10);

	     			// power on
	     			PWR_ON();

	     			HAL_Delay(10);

	     			// SCLK to high
	     			pullstart=1;
	     			MX_SPI1_Init();

	     			// SEN to high
	     			HMC832_SEN_Set();

	     			HAL_SPI_MspInit(&hspi1);

	     			pullstart=2;
	     			HAL_SPI_MspInit(&hspi1);

	     			HAL_Delay(100);

	     			DET_ENABLE();

	     			division = (int) hexatofloat((RxData + 4), 0);
	     			if(division > (MAXINDX-100))
	     				division = MAXINDX-100;

	     			// initizlize channel value
	     			// create list and data input from RX
	     			if(head != NULL) delete(head);
	     			head = malloc(sizeof(NODE));
	     			head->next = NULL;

#ifdef POINTERON
	     			if(chhead != NULL) deleteCH(chhead);
	     			chhead = malloc(sizeof(CHANNELNODE));
	     			chhead->next = NULL;
#endif

	     			for(int i=0; chanSize > i; i++)
      				{
     					U8 ch = RxData[7+i*7];
     					float st = hexatofloat((RxData + (8+i*7)), 2);
     					float sp = hexatofloat((RxData + (11+i*7)), 2);
     					addLast(head, ch, st, sp);

#ifdef POINTERON
     					addLastCH(chhead, ch, st, sp, division);
#endif

      				}
	     			blockCtrl = MEAS_BLOCK;
	     			vergine = 1;

			} // end of if(chanSize >0)
			//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET); // LED debug pin

		} // end of FnCode == 0x09


		/////////////////////////////////////////////////////////
		//. Stop flag
		/////////////////////////////////////////////////////////
		else if (FnCode == 0x0A) {
			// TxLen check value later
			// switch on for analog measure start
			test_var = 0;
			valIndex = 0;

			// clear memory
			if(head != NULL) {
				delete(head);
				head = NULL;
			}

	    		pullstart=0;
	    		MX_SPI1_Init();
	    		HAL_SPI_MspDeInit(&hspi1);
	    		DET_DISABLE();



			PWR_OFF();

			blockCtrl = NO_MEAS_BLOCK;
			firstLoop = 1;
			uartit_on=1;


			// HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET); // LED debug pin

			// end of FnCode == 0x0A
		} else {
			return 0;
		}
		// wait until all execution finished
		TxData[4] = 0x00;


		// end of myID == 0x01 or myID == 0x00
	} else {
		return 0;
	}
	return 1;
}

void comm_rs485_rtxProcess() {

	FnErrCode = 0x00;
	ErrStatus = 0x00;
	RxFlag = 1;
	rxiter=0;

	for(int i=0; RxSize >i; i++)
	{

		std_rs485_rxGetId(i);

		// if there are one frame size left to process


		////////////////////////////////////////////////////
		//// x68 protocol start
		////////////////////////////////////////////////////
		//else if (RxProtocol == 0x68)
		if (RxData[i+0] == 0x68 && ( myID == RxData[i+2] || 0x00 == RxData[i+2] )) {
			len = RxData[i+1];
			if(len < RxSize -i)
			{
				//  exclude x86 and last crc value: -2
				// if (x68_rxCrcCheck8((U8) (RxSize - 3)))  // working on #1 board
				if (x68_rxCrcCheck8(len, i)) {

					if (x68_rxProcess()) {
						;
					} 
				}
				rxiter=i;
			} else { // end of frame size rest left to copy for next process
				memcpy(RxData, RxData+rxiter, RxSize-i);
				rxiter = RxSize-i;
				i = RxSize; // terminate for loop
			}

		}
	}
}









/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	int i=0;
	for(i=0; i<len; i++)
		ITM_SendChar((*ptr++));
	HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY);
	return len;
}

uint8_t count=0;


/*
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
       	//memcpy(&user_buff[0], & adc_buff[0], NO_SAMPLE/2);

	memcpy(&user_buff[0], &adc_buff[0], sizeof(uint16_t)*MAXAVG/2);

	ADC_HalfConvCpltFlag = 1;

}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

	//memcpy(&user_buff[NO_SAMPLE/2], & adc_buff[NO_SAMPLE/2], NO_SAMPLE/2);
	//memcpy(&user_buff[0], &adc_buff[0], MAXAVG);

	memcpy(&user_buff[0], &adc_buff[0], sizeof(uint16_t)*MAXAVG);

	ADC_ConvCpltFlag = 1;

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
      	if(htim->Instance == TIM6)
      	{
      		memcpy(&user_buff[0], &adc_buff[0], MAXAVG);
      		ADC_ConvCpltFlag = 1;

      	}
}
*/

static void My_ADC1_Init(void)
{
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    /** Common config */
    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV2;     // 필요시 DIV8→DIV2로
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait      = DISABLE;
    hadc1.Init.ContinuousConvMode    = ENABLE;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc1.Init.Overrun               = ADC_OVR_DATA_PRESERVED;   // 데이터 보존 권장
    hadc1.Init.LeftBitShift          = ADC_LEFTBITSHIFT_NONE;    // 정렬 대체 개념
    hadc1.Init.OversamplingMode      = DISABLE;

    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /** Configure the ADC multi-mode */
    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Regular Channel (PA0 = ADC1_IN16) */
    sConfig.Channel                  = ADC_CHANNEL_16;
    sConfig.Rank                     = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime             = ADC_SAMPLETIME_8CYCLES_5; // 2.5→8.5로 여유
    sConfig.SingleDiff               = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber             = ADC_OFFSET_NONE;
    sConfig.Offset                   = 0;
    sConfig.OffsetSignedSaturation   = DISABLE;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

void HighSpeed_Pin_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Port Clock Enable */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* FC1 = PD6, FC2 = PD14 */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;     // Push-pull output
    GPIO_InitStruct.Pull = GPIO_NOPULL;             // No pull-up/down
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  // ★ 매우 중요: 고속 출력
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* FC3 = PB3, BIT_EN = (예: PB15) */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 초기값: 모두 LOW */
    GPIOD->BSRR = (1U << (6 + 16)) | (1U << (14 + 16));
    GPIOB->BSRR = (1U << (3 + 16));
}

/* ======== DWT Cycle Counter Setup ======== */
static inline void DWT_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // DWT enable
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // start counting
}

static inline uint32_t DWT_GetCycles(void)
{
  return DWT->CYCCNT;
}


void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        awd_ticks = DWT->CYCCNT;
         //x68_txFreq(0, 999.99);
         //x68_txProcess();
        // HAL_GPIO_WritePin(GPIOD, LED_CH6_Pin, GPIO_PIN_SET);
        //printf("AWD configured on ADC1_IN16, High=%u (%.2f V)\r\n", 0.0, 0.0);
        awd_flag = 1;
    }
    //printf("AWD configured on ADC1_IN16, High=%u (%.2f V)\r\n", 0.0, 0.0);
}

void ADC1_AWD_Config_Code(float vref, float thr_v)
{
    ADC_AnalogWDGConfTypeDef awd = {0};

    // 1. Adc raw count로 허용치 계산 (0~4095)
    uint32_t high_th_count = (uint32_t)((thr_v / vref) * 4095.0f);

    awd.WatchdogMode  = ADC_ANALOGWATCHDOG_SINGLE_REG;
    awd.Channel       = ADC_CHANNEL_16;       // PA0
    awd.ITMode        = ENABLE;
    awd.HighThreshold = high_th_count;
    awd.LowThreshold  = 0;                    // 0 ~ High

    if (HAL_ADC_AnalogWDGConfig(&hadc1, &awd) != HAL_OK) {
        Error_Handler();
    }

    // 2. NVIC에서 인터럽트 활성화
    HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);

    // 3. 정규 변환 + AWD 인터럽트 시작
    if (HAL_ADC_Start_IT(&hadc1) != HAL_OK) {
        Error_Handler();
        //printf("AWD configured on ADC1_IN16, High=%u (%.2f V)\r\n", high_th_count, thr_v);
    }
    //printf("AWD configured on ADC1_IN16, High=%u (%.2f V)\r\n", high_th_count, thr_v);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	int preRx_indx;
    	int setmyID = 0;

    	pullstart = 0;
    	uartit_on = 0;
 
	uint8_t id1=0, id2=0;


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* ---------- 함수 선언 ---------- */
  void MX_USART3_UART_Init(void);
  /*
  void ADC_to_Input_Mode(void);
  void Input_to_ADC_Mode(void);
  */
  /*
  static inline void DWT_Init(void);
  static inline uint32_t DWT_GetCycles(void);
   */

      	// us delay init
	DWT_Delay_Init();

	//HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET); // OK pin
	//HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_RESET); // FL pin

	HAL_UART_Receive_IT(&huart1, &Rx_data, 1);// activate uart rx interrupt every time receiving 1 byte
	//HAL_TIM_Base_Start_IT(&htim4);

	TIM4->CNT = 0;

	test_var = 0;
	// HAL_TIM_Base_Start_IT(&htim4); is not working
	HAL_TIM_Base_Start_IT(&htim3); // for STM32H723
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

	//ST7789_Init();

	//// to initialize address index

	////
	RxFlag = 0;

	//// should get from eeprom later, firstly

	/// TEST PURPOSE, register value serialize to check
	valTestInit();
	//id on eeprom

	// myID = 0x01;

	HAL_UART_Receive_IT(&huart1, &Rx_data, 1);// activate uart rx interrupt every time receiving 1 byte
	//HAL_TIM_Base_Start_IT(&htim4);

	//TIM4->CNT = 0;
   	setmyID = myID = 1;
   	SPECIAL_MX_GPIO_Init(setmyID);
    DWT_Init();
    HighSpeed_Pin_Init();     // ★ 여기서 고속핀 초기화
    MX_ADC1_Init();
    // ADC1_AWD_Config_Code(3.3f, 1.6f);

    //printf("=== UART printf test (115200 baud) ===\r\n");



	/*Configure GPIO pin Output Level */





	 HAL_GPIO_WritePin(GPIOC, LED_CH3_Pin|LED_CH5_Pin, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOA, LED_CH2_Pin, GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOD, LED_CH4_Pin|LED_CH6_Pin|BIT_EN_Pin, GPIO_PIN_RESET);
     HAL_Delay(3000);
	 HAL_GPIO_WritePin(GPIOC, LED_CH3_Pin|LED_CH5_Pin, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOA, LED_CH2_Pin, GPIO_PIN_SET);
     HAL_GPIO_WritePin(GPIOD, LED_CH1_Pin | LED_CH4_Pin|LED_CH6_Pin, GPIO_PIN_SET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOB, FC3_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOD, FC1_Pin|FC2_Pin, GPIO_PIN_RESET);




	/*
    	////////////////////////////////////// hardware ID check ////////////////////////////////
   	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6))
   	{
   		id1 = 1;
   	}

   	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7))
   	{
   		id2 = 1;
   	}

   	if(id1 && id2 )
   	{
   		setmyID = 0;
   	} else {
   		if(id1)
   			setmyID = 1;
   		else if(id2)
   			setmyID = 2;
   	}

    */

   	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);


	for(int i=0; myID > i; i++ )
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_Delay(100);

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_Delay(100);
	}


   	//HAL_UART_Receive_IT(&huart1, &Rx_data, 1);// activate uart rx interrupt every time receiving 1 byte
   	///////////////////////////////////////// end of myid set ////////////////////////////////////


	/////////////////////////// HMC832 HMC SPI mode ///


	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);	//HMC832 power reset
	HAL_Delay(500);

	// rtsLock(0, 0); // unlock all for initialize

	pullstart=0;
	HAL_SPI_MspInit(&hspi1);

	// SPI2 LE disable
	PE43712_SEN_Set();




	//HAL_TIM_Base_Start(&htim8);

	HAL_Delay(100);
	// HMC832_SEN_Set();

	HAL_Delay(100);

	/// for debugging


	//https://github.com/Embedfire-lwip/ebf_lwip_tutorial_code_stm32h743_pro_v/blob/master/lwip_udp_iperf/Libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c

	//////////////// DMA ADC STAR /////////////////////
	/*
	HAL_SYSCFG_EnableBOOST();
	HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA0, SYSCFG_SWITCH_PA0_CLOSE);

	while(HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY,ADC_SINGLE_ENDED ) != HAL_OK);
	*/



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	// rxon_and_alloff();

	/*
   	   for(int i=0; MAXAVG > i; i++ )
   	   {
	   adc_buff[i]=300;
   	   }
*/

      	while (1)
      	{

		// initialize index on HAL_UART_RxCpltCallback
		// Rx_indx = 0;
		xferDispOn = 0;

		////////////// update data memory from buffer ////////////
		if (Rx_indx > 0) {
			//ST7789_WriteString(3, 150, "X68   Rx_idx ", Font_16x26, YELLOW, BLUE);
			do {
				preRx_indx = Rx_indx;
				HAL_Delay(10);
			} while (Rx_indx > preRx_indx);

			//if(TIM4->CNT > 200)
			//{
			memcpy(rxBuffer, Rx_Buffer, Rx_indx);
			//memcpy(RxData, Rx_Buffer, Rx_indx);
			memcpy(RxData+rxiter, Rx_Buffer, Rx_indx);
			xferDispOn = 1;
			RxSize = dataSize = Rx_indx;
			Rx_indx = 0;
			//TIM4->CNT = 0;

			comm_rs485_rtxProcess();
			//}
		}
		////////////////////////////////////////////////
		//   ANALOG MEASURE ////////////////////////////
		////////////////////////////////////////////////


		//// calibrating

		/*
		// writing ID
		if (blockCtrl == MEAS_BLOCK) {

			// execution control
			blockCtrl = PAUSE_BLOCK;  //for every start require signal


			NODE *curr = head->next;
			while(curr != NULL)
			{
				if( division > 1)
				{
					if( firstLoop == 1)
					{
						valTestInit();
						midcode[curr->ch]=0x00;

						//PE43712_writeReg(0x00, midcode[curr->ch]); // ATTN level control
						PE43712_writeReg(0x00, 0x01); // ATTN level control
						rfSensing(curr->ch, curr->stFreq, curr->spFreq);
						contBinSearch(maxSensVal[curr->ch], curr->ch);

					} else {

						// PE43712_writeReg(0x00, midcode[curr->ch]);
						PE43712_writeReg(0x00, 0x01);
						//PE43712_writeReg(0x00, 0x7F);

						// end of atten level adjust
						rfSensing(curr->ch, curr->stFreq, curr->spFreq); // ATTN level control
						contBinSearch(maxSensVal[curr->ch], curr->ch);

					}

				}
				curr = curr->next;
			}

			if( firstLoop == 1)
				firstLoop = 0;


			// execution control , master wait slave finish
			blockCtrl = PAUSE_BLOCK;
			// blockCtrl = PAUSE_BLOCK; /// stop without infinite loop
			// blocking for waiting continue

		}
		*/

		if (blockCtrl == MEAS_BLOCK) {

			// execution control
			blockCtrl = PAUSE_BLOCK;  //for every start require signal


			NODE *curr = head->next;
			while(curr != NULL)
			{
				if( division > 1)
				{
						valTestInit();
						midcode[curr->ch]=0x00;

						//PE43712_writeReg(0x00, midcode[curr->ch]); // ATTN level control
						PE43712_writeReg(0x00, 0x01); // ATTN level control
						rfSensing(curr->ch, curr->stFreq, curr->spFreq);
						contBinSearch(maxSensVal[curr->ch], curr->ch);

				}
				curr = curr->next;
			}

			if( firstLoop == 1)
				firstLoop = 0;


			// execution control , master wait slave finish
			blockCtrl = PAUSE_BLOCK;
			// blockCtrl = PAUSE_BLOCK; /// stop without infinite loop
			// blocking for waiting continue

		}

		//// Sync ID1 and ID2 for
		/*
		*/

		// light on off
		if(uartit_on)
     		{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
	  		HAL_Delay(100);

	  		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
	  		HAL_Delay(100);

	  		uartit_on = 0;

     		} else {
	 		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
     		}

		uartit_on = 0;


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SPI2
                              |RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 16;
  PeriphClkInitStruct.PLL2.PLL2P = 20;
  PeriphClkInitStruct.PLL2.PLL2Q = 4;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  My_ADC1_Init();

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

      	if(pullstart >= 1)
      	{
      		hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
      	} else {
      		hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
      	}
      	if (HAL_SPI_Init(&hspi1) != HAL_OK)
      	{
	    	Error_Handler();
      	}


  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 400;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_4|LED_CH3_Pin|LED_CH5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_8|LED_CH2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|FC3_Pin|BIT_ENB4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|FC2_Pin|LED_CH1_Pin|BIT_EN_Pin
                          |LED_CH6_Pin|LED_CH4_Pin|GPIO_PIN_5|FC1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC4 LED_CH3_Pin LED_CH5_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4|LED_CH3_Pin|LED_CH5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA8 LED_CH2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_8|LED_CH2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 FC3_Pin BIT_ENB4_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|FC3_Pin|BIT_ENB4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 FC2_Pin LED_CH1_Pin BIT_EN_Pin
                           LED_CH6_Pin LED_CH4_Pin PD5 FC1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_8|FC2_Pin|LED_CH1_Pin|BIT_EN_Pin
                          |LED_CH6_Pin|LED_CH4_Pin|GPIO_PIN_5|FC1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void SPECIAL_MX_GPIO_Init(uint8_t id)
{
      	GPIO_InitTypeDef GPIO_InitStruct = {0};

      	if(id==1)
      	{
      		/*Configure GPIO pin Output Level */
      		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_4, GPIO_PIN_RESET);
      		GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4;

      		/*
      		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_6
      				|GPIO_PIN_8, GPIO_PIN_RESET);
      		GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_6
			|GPIO_PIN_8;
      		*/


      	} else if(id == 2){
      		/*Configure GPIO pin Output Level */
      		/*
      		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_7
				|GPIO_PIN_9, GPIO_PIN_RESET);

      		GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_7
			|GPIO_PIN_9;
      		*/
      		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_4, GPIO_PIN_RESET);
      		GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4;


      	} else if( id == 0){
      		return;
      	}


      	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      	GPIO_InitStruct.Pull = GPIO_NOPULL;
      	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}




//Interrupt callback routine
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	uint8_t i;

	// ST7789_WriteString1(3, 150, "0X68 ON", Font_16x26, YELLOW, BLUE); not working to slow
	if (huart->Instance == USART1) //current UART
	{
		if (Rx_indx == 0) {
			for (i = 0; i < MAXSIZE - 1; i++)
				Rx_Buffer[i] = 0;
			// TIM4->CNT=0;
		}   //clear Rx_Buffer before receiving new data
		if (Rx_indx < MAXSIZE) {
			Rx_Buffer[Rx_indx++] = Rx_data;    //add data to Rx_Buffe
		} else {
			Rx_indx = 0;
			//Rx_Buffer[Rx_indx++]=Rx_data;
			Transfer_cplt = 1;
		}

		// initialize timer and restart uart receive//
		// HAL_TIM_Base_Start_IT(&htim4);
		//TIM4->CNT = 0; //__HAL_TIM_SetCounter(&htim4, 0); // set counter to 0
		HAL_UART_Receive_IT(&huart1, &Rx_data, 1);
		//ST7789_WriteString1(3, 150, "0x86", Font_16x26, YELLOW, BLUE);
	}

}



int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart3, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
      	/* User can add his own implementation to report the HAL error return state */
      	__disable_irq();
      	while (1)
      	{
      	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
      	/* User can add his own implementation to report the file name and line number,
ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

