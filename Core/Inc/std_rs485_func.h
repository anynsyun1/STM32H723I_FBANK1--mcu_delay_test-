///////// chec check for RX data from the arg len
U16 std_rs485_rxCrcCheck(U16 len) {
	U16 crcRslt=0xFFFF;

	crcRslt = crc16_cal(RxData, len);

	if(crcRslt == (RxData[RxSize-2] << 8 | RxData[RxSize-1] ))
	{
		return crcRslt;
	} else
		return 0;
}

#define IDLOC 0

U8 std_rs485_rxGetId(int i )
{
	///////////////////////////////////////////
	// ID check
	if(RxSize > IDLOC+i) {
		RxID = RxData[IDLOC+i];
		return RxID;
	} else {
		return 0;
	}
}

/*
U8 std_rs485_rxGetFcode()
{
	/////////////////////////////////////////////
	// FUNC check
	if( RxSize > FNLOC) {
		for(U8 i=0; i < FUNCLIST; i++)
		{
			//to find out listed function codes
			if(RxData[FNLOC] == fcl[i])
			{

				FnCode = RxData[FNLOC];
				findex = i;

				return FnCode;
			}
			if(i == (FUNCLIST-1))
			{
				// error, no func code found
				FnErrCode = RxData[FNLOC] | ERR_MASK;
				ErrStatus = 0x01;
				TIM4->CNT = timeOut;

				return 0;
			}

		}
		return 0;
	} else {
		return 0;
	}
}
*/

/*
U16 std_rs485_getAddr()
{
	////////////////////////////////////////////////////////
	// process address /////////////////////////////////////
	if(RxSize > ADLOC) {

		U16 tmpaddr=0;
		tmpaddr = (U16)(RxData[ADLOC] << 8| RxData[ADLOC+1]);

		/// address is ok
		// to check if adl array index is under boundry
		if(findex < FUNCLIST && findex >=0)
		{
			if(tmpaddr >= adl[findex][0] && tmpaddr <= adl[findex][1] )
			{
				RxAddr = tmpaddr;
				return RxAddr;

			} else {
				// error: no register address
				// there should be error
				return 0;
			}
		} else {
			// no funtioncode
			return 0;
		}
	} else {
		/// address loc is not exist
		return 0;
	}
}
*/

/*
U16 std_rs485_getRxDataLocNLen(U8 *pRxDt)
{
	// get address index
	for(U8 i=0; i < REG_SIZE; i++)
	{
		if(AddrMap[i].addr == RxAddr)
		{

			//AddrMap[i].addrPtr;
			addrIdx = i;
			break;
		}
	}
	////////////////////////////////////////////////////////
	// process word length /////////////////////////////////
	if(RxSize > CNLOC ) {
		if(FnCode >= READ_DIST && FnCode <= READ_INPUT  ) {

			RxDLen=0;
			RxDLen = (U16)(RxData[CNLOC] << 8 | RxData[CNLOC+1]);

			RxDLen=RxDLen*2;
			TxDLen = RxDLen;
			pRxDt = 0x00;

			return RxDLen;
		} else if ( FnCode == WRITE_SINGLE) {
			TxDLen = 1;
			RxDLen=1*2;
			pRxDt = RxData+CNLOC;
			// copy 1 byte to right address
			memcpy(valInAddr+addrIdx, RxData+CNLOC+1, 1);

			return RxDLen;
		} else if ( FnCode == WRITE_MULTIPLE )
		{
			RxDLen = (U16)(RxData[CNLOC] << 8 | RxData[CNLOC+1]);
			TxDLen = RxDLen;
			RxDLen=RxDLen*2;
			pRxDt = RxData+CNLOC+4;

			// copy data to the right address.
			for(U8 i=0; i < RxDLen; i++)
			{
				memcpy(valInAddr+addrIdx+i, RxData+CNLOC+3+i, 1);
			}



			return RxDLen;
		} else {
			// no function code
			return 0;
		}

	} else {
		// incomplete data size
		return 0;
	}

}
*/

/*

void std_rs485_rxProcess()
{
	U8 *pRxDt=0;
	//rs485_rxGetFcode();
	//rs485_getAddr();
	//rs485_getRxDataLocNLen(pRxDt);
	if(std_rs485_rxGetFcode())
	{
		if(std_rs485_getAddr())
		{
			if(std_rs485_getRxDataLocNLen(pRxDt))
			{
				RxFlag = 1;
			} else {

				FnErrCode =  ERR_MASK | FnCode;
				ErrStatus = 0x03;
				RxFlag = 0;
			}
		} else {

			FnErrCode =  ERR_MASK | FnCode;
			ErrStatus = 0x02;
			RxFlag = 0;
		}
	} else {
		FnErrCode = RxData[FNLOC] | ERR_MASK;
		ErrStatus = 0x01;
		RxFlag = 0;
	}
}




void std_rs485_txHeadConst() {
	TxLen = 0;
	TxData[TxLen++] = myID;
	TxData[TxLen++] = FnCode;

	if(FnCode >= READ_DIST && FnCode <= READ_INPUT)
	{
		TxData[TxLen++] = (U8)(RxDLen);

	} else {
		// WD size for address
		TxData[TxLen++] = RxAddr >> 8;
		TxData[TxLen++] = RxAddr & 0xFF;

		//TxLen++;
		if(FnCode == WRITE_MULTIPLE)
		{
			// WD size for wd count
			TxData[TxLen++] = (RxDLen/2) >> 8;
			TxData[TxLen++] = (RxDLen/2) & 0xFF;

			// byte size for byte count

			//TxLen++;
		}

	}
}

*/
/*
void std_rs485_txFrameBodyConst() {
	//U8  addrIdx=0;

	/// getting data start address point
	for(U8 i=0; i < REG_SIZE; i++)
	{
		if(AddrMap[i].addr == RxAddr)
		{
			//addrPtr = AddrMap[i].addrPtr;
			addrIdx = i;
			break;
		}
	}

	if(FnCode >= READ_DIST && FnCode <= WRITE_SINGLE) {

		if(FnCode >= READ_DIST && FnCode <= READ_INPUT)
		{
			for(U8 i=0; i < TxDLen; i++)
			{
				//TxData[3+i] = (U8 *)(addrPtr+i);
				//memcpy(TxData+TxLen+i, addrPtr+i, 1);
				memcpy(TxData+TxLen+i, valInAddr+addrIdx+i, 1);

			}
			TxLen += TxDLen;



		} else if(FnCode == WRITE_SINGLE)
		{

			// copy single write data to Tx
			memcpy(TxData+TxLen, RxData+CNLOC, 1);
			TxLen++;
			memcpy(TxData+TxLen, RxData+CNLOC+1, 1);
			TxLen++;
		}
	}
}
*/

void std_rs485_txCRCConst() {
	U16 crcRslt=0xFFFF;

	crcRslt = crc16_cal(TxData, TxLen);
	TxData[TxLen] = crcRslt >> 8;
	TxData[++TxLen] = crcRslt & 0xFF;

}




void std_rs485_sendErr() {

	TxLen = 0;
	TxData[TxLen++] = myID;
	TxData[TxLen++] = FnErrCode;
	TxData[TxLen++] = ErrStatus;

	std_rs485_txCRCConst();

	// send error message to host
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_UART_Transmit(&huart1, (uint8_t *)(&TxData), TxLen+1, 100);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

}


void std_rs485_txProcess()
{
	//std_rs485_txHeadConst();
	//std_rs485_txFrameBodyConst();
	std_rs485_txCRCConst();

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_UART_Transmit(&huart1, (uint8_t *)(&TxData), TxLen+1, 100);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

}

