/*
 * max30102.c
 *
 *  Created on: Feb 9, 2023
 *      Author: Eng.Hazem
 */


#include "max30102.h"
extern I2C_HandleTypeDef hi2c1;
uint8_t space[]=" \n\r";
extern float Final_result;
DC_FILTER_T dcFilterIR = {0};
DC_FILTER_T dcFilterRed = {0};
MEAN_DIFF_FILTER_T meanDiffIR = {0};
BUTTERWORTH_FILTER_T lpbFilterIR = {0};
uint8_t redLEDCurrent = 0;
float lastREDLedCurrentCheck = 0;
float currentBPM=0;
float oldcurrentBPM=0;
float valuesBPM[10] = {0};
float valuesBPMSum = 0;
uint8_t valuesBPMCount = 0;
uint8_t bpmIndex = 0;
uint32_t lastBeatThreshold = 0;

float irACValueSqSum = 0;
float redACValueSqSum = 0;
uint16_t samplesRecorded = 0;
uint16_t pulsesDetected = 0;
float currentSpO2Value = 0;
float OldSpO2Value = 0;
LEDCurrent IrLedCurrent;
int flag=0;
PULSE_STATE_MACHINE currentPulseDetectorState = PULSE_IDLE;

#if DEBUG_MODE_UART==OPEN
uint8_t arr_mode[]="Error happen in the mode \n\r";
uint8_t arr_mode2[]="Error happen in the mode re2 \n\r";
uint8_t arr_modecurr[]="every thing is ok in the mode \n\r";
uint8_t arr_sampling[]="Error happen in sampling \n\r";
uint8_t arr_sampling2[]="Error happen in sampling re2 \n\r";
uint8_t arr_samplingcurr[]="every thing is ok in sampling\n\r";
uint8_t arr_pulse[]="Error happen in set pulse \n\r";
uint8_t arr_pulse2[]="Error happen in set pulse ret2 \n\r";
uint8_t arr_pulsecurr[]="every thing is ok in set pulse \n\r";
uint8_t test[]="The value is ready \n\r";
#endif
extern UART_HandleTypeDef huart5;
uint8_t Max30102_readRegister(uint8_t reg, uint8_t* value)
{
	HAL_StatusTypeDef retStatus;
	uint8_t Return_value=0;
	uint8_t Address=(I2C_SLAVE_ID|I2C_WRITE);

	retStatus = HAL_I2C_Master_Transmit(&hi2c1,Address, &reg, 1, HAL_MAX_DELAY);
	if(retStatus!=HAL_OK)
	{
		return -1;
	}
	Address=(I2C_SLAVE_ID|I2C_READ);
	retStatus = HAL_I2C_Master_Receive(&hi2c1, Address, &Return_value, 1, HAL_MAX_DELAY);

	if(retStatus!=HAL_OK)
		{
			return -1;
		}

	*value=Return_value;

	return 1;
}


HAL_StatusTypeDef Max30102_writeRegister(uint8_t reg, uint8_t value)
{
	HAL_StatusTypeDef retStatus;
	uint8_t buf[2];
	buf[0] = reg;
	buf[1] = value;

	uint8_t address = (I2C_SLAVE_ID | I2C_WRITE);
	retStatus = HAL_I2C_Master_Transmit(&hi2c1, address, buf, 2, HAL_MAX_DELAY);

	return retStatus;
}



void Max30102_Init(void)
{

	/******************set configration for FIFO*************************/
			/*****************************Sample Averaging**********************/
			Max30102_writeRegister(FIFO_CONFIG, 0x0F);
			/**********with out interrupt********************************/
			Max30102_writeRegister(INT_ENABLE_1, 0x00);
	/***********************MODE IS SpO2 mode**************************************************/
	int8_t readStatus = 0;
	uint8_t readResult;
	HAL_StatusTypeDef Status;

	/****************SETING SAMPLING RATE****************************************************/
		readStatus=Max30102_readRegister(SPO2_CONFIG, &readResult);
		if(readStatus==-1)
			{
			#if DEBUG_MODE_UART==OPEN
			HAL_UART_Transmit(&huart5, arr_sampling, sizeof(arr_sampling), 500);
			#endif
				return;
			}else
			{
				#if DEBUG_MODE_UART==OPEN
				HAL_UART_Transmit(&huart5, arr_samplingcurr, sizeof(arr_samplingcurr), 500);
				#endif
			}
		readResult &= ~(0x1C << 0);
		readResult = readResult | (0x01 << 2);

		if( Max30102_writeRegister(SPO2_CONFIG, readResult) != HAL_OK){
			#if DEBUG_MODE_UART==OPEN
			HAL_UART_Transmit(&huart5, arr_sampling2, sizeof(arr_sampling2), 500);
			#endif
				return;
			}else
			{
				#if DEBUG_MODE_UART==OPEN
				HAL_UART_Transmit(&huart5, arr_samplingcurr, sizeof(arr_samplingcurr), 500);
				#endif
			}
		/*********set pulse width**********************************/
			readStatus = Max30102_readRegister(SPO2_CONFIG, &readResult);
				if( readStatus == -1){
					#if DEBUG_MODE_UART==OPEN
					HAL_UART_Transmit(&huart5, arr_pulse, sizeof(arr_pulse), 500);
					#endif
					return;
				}else
				{
					#if DEBUG_MODE_UART==OPEN
					HAL_UART_Transmit(&huart5, arr_pulsecurr, sizeof(arr_pulsecurr), 500);
					#endif
				}

				readResult &= ~(0x03 << 0);

				/************pluse width is 411**********************/
				readResult = readResult | (0x03 << 0);
				if( Max30102_writeRegister(SPO2_CONFIG, readResult) != HAL_OK){
					#if DEBUG_MODE_UART==OPEN
					HAL_UART_Transmit(&huart5, arr_pulse2, sizeof(arr_pulse2), 500);
					#endif
						return;
					}else
					{
						#if DEBUG_MODE_UART==OPEN
						HAL_UART_Transmit(&huart5, arr_pulsecurr, sizeof(arr_pulsecurr), 500);
						#endif
					}

				Max30102_setLedCurrent(RED_LED, 0xff);
				  Max30102_setLedCurrent(IR_LED,5);
				 Max30102_resetFifo();
	/**read the mode register to save the the last bits and change in the mode bit only****/
	readStatus=Max30102_readRegister(MODE_CONFIG, &readResult);
	if(readStatus==-1)
	{
		#if DEBUG_MODE_UART==OPEN
		HAL_UART_Transmit(&huart5, arr_mode, sizeof(arr_mode), 500);
		#endif
		return;
	}else
	{
		#if DEBUG_MODE_UART==OPEN
		HAL_UART_Transmit(&huart5, arr_modecurr, sizeof(arr_modecurr), 500);
		#endif
	}
	/****set the mode bits to zero *****/
	readResult &= ~(0x07 << 0);
	/****set the mode to spo2*******/
	/**011 SpO2 mode**/
	readResult=(readResult|0x03);
	/**write the new mode in the register******/
	Status=Max30102_writeRegister(MODE_CONFIG,readResult);
	if(Status!=HAL_OK){
		#if DEBUG_MODE_UART==OPEN
		HAL_UART_Transmit(&huart5, arr_mode2, sizeof(arr_mode2), 500);
		#endif
		return;
	}else
	{
		#if DEBUG_MODE_UART==OPEN
		HAL_UART_Transmit(&huart5, arr_modecurr, sizeof(arr_modecurr), 500);
		#endif
	}





}



FIFO_LED_DATA Max30102_readFifo(void)
{
	uint8_t address;
	uint8_t buf[12]={0};
	uint8_t numBytes = 6;
	FIFO_LED_DATA fifoData;


	buf[0] = FIFO_DATA;

	address = (I2C_SLAVE_ID | I2C_WRITE);

	HAL_I2C_Master_Transmit(&hi2c1, address, buf, 1, HAL_MAX_DELAY);

	address = (I2C_SLAVE_ID | I2C_READ);
	HAL_I2C_Master_Receive(&hi2c1, address, buf, numBytes, HAL_MAX_DELAY);

	fifoData.irLedRaw = 0;
	fifoData.redLedRaw = 0;
	#if DEBUG_MODE_UART==OPEN
	HAL_UART_Transmit(&huart5, test, sizeof(test), 500);
	#endif

	fifoData.irLedRaw = (((uint32_t)buf[3] << 16)|((uint32_t)buf[4] << 8) | ((uint32_t)buf[5] << 0))&0x3ffff;
	fifoData.redLedRaw =(((uint32_t)buf[0] << 16)|((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 0))&0x3ffff;
	return fifoData;
}
MAX30102 Max30102_HeartRate_Value(FIFO_LED_DATA m_fifoData)
{

	MAX30102 result = {
	/*bool pulseDetected*/ false,
	/*float heartBPM*/ 0.0,
	/*float irCardiogram*/ 0.0,
	/*float irDcValue*/ 0.0,
	/*float redDcValue*/ 0.0,
	/*float SaO2*/ currentSpO2Value,
	/*uint32_t lastBeatThreshold*/ 0,
	/*float dcFilteredIR*/ 0.0,
	/*float dcFilteredRed*/ 0.0,

  };


	dcFilterIR = dcRemoval( (float)m_fifoData.irLedRaw, dcFilterIR.w, ALPHA );
	dcFilterRed = dcRemoval( (float)m_fifoData.redLedRaw, dcFilterRed.w, ALPHA );
	float meanDiffResIR = meanDiff( dcFilterIR.result, &meanDiffIR);
	lowPassButterworthFilter( meanDiffResIR/*-dcFilterIR.result*/, &lpbFilterIR );
	irACValueSqSum += dcFilterIR.result * dcFilterIR.result;
	redACValueSqSum += dcFilterRed.result * dcFilterRed.result;
	samplesRecorded++;

	if( detectPulse( lpbFilterIR.result ) && samplesRecorded > 0 )
	{
		result.pulseDetected=true;
		pulsesDetected++;
		flag=1;

		float ratioRMS = log( sqrt(redACValueSqSum/samplesRecorded) ) / log( sqrt(irACValueSqSum/samplesRecorded) );

		//This is my adjusted standard model, so it shows 0.89 as 94% saturation. It is probably far from correct, requires proper empircal calibration
		//float ratioRMS=(((int)redACValueSqSum/(int)dcFilterRed)/((int)irACValueSqSum/(int)dcFilterIR));
		currentSpO2Value=115.0-17.0*ratioRMS;



		//result.SpO2 = currentSpO2Value;

		if(currentSpO2Value>=0&&currentSpO2Value<=100.0){
			result.SpO2 = currentSpO2Value;
			OldSpO2Value=currentSpO2Value;
		}else if(currentSpO2Value<80.0){
			result.SpO2 =80.0;
		}else if(currentSpO2Value>100.0){
			currentSpO2Value=OldSpO2Value;
		}


		if( pulsesDetected % 8 == 0)
		{
			irACValueSqSum = 0;
			redACValueSqSum = 0;
			samplesRecorded = 0;
		}
	}

	balanceIntesities( dcFilterRed.w, dcFilterIR.w );
	  if(currentBPM<=100.0){
				result.heartBPM = currentBPM;
				oldcurrentBPM=currentBPM;
			}else if(currentBPM>100.0)
			{

				result.heartBPM = oldcurrentBPM;
				currentBPM=currentBPM-20;
			}
	result.irCardiogram = lpbFilterIR.result;
	result.irDcValue = dcFilterIR.w;
	result.redDcValue = dcFilterRed.w;
	result.lastBeatThreshold = lastBeatThreshold;
	result.dcFilteredIR = dcFilterIR.result;
	result.dcFilteredRed = dcFilterRed.result;

	return result;
}


bool detectPulse(float sensor_value)
{
  static float prev_sensor_value = 0;
  static uint8_t values_went_down = 0;
  static uint32_t currentBeat = 0;
  static uint32_t lastBeat = 0;

  if(sensor_value > 2000)
  {
	  //HAL_UART_Transmit(&huart5, "TRUE\n\r", sizeof("TRUE\n\r"), 100);
    currentPulseDetectorState = PULSE_IDLE;
    prev_sensor_value = 0;
    lastBeat = 0;
    currentBeat = 0;
    values_went_down = 0;
    lastBeatThreshold = 0;
    return false;
  }

  switch(currentPulseDetectorState)
  {
    case PULSE_IDLE:

      if(sensor_value >= 100) {
    	  //HAL_UART_Transmit(&huart5, "TRUE1\n\r", sizeof("TRUE1\n\r"), 100);
        currentPulseDetectorState = PULSE_TRACE_UP;
        values_went_down = 0;
      }
      break;

    case PULSE_TRACE_UP:

      if(sensor_value > prev_sensor_value)
      {
    	  //HAL_UART_Transmit(&huart5, "TRUE2\n\r", sizeof("TRUE2\n\r"), 100);
        currentBeat = millis();
        lastBeatThreshold = sensor_value;
      }
      else
      {
    	 // HAL_UART_Transmit(&huart5, "TRUE3\n\r", sizeof("TRUE3\n\r"), 100);
    	uint32_t beatDuration = currentBeat - lastBeat;
        lastBeat = currentBeat;

        float rawBPM = 0;
        if(beatDuration > 0)
          rawBPM = 60000.0 / (float)beatDuration;

        valuesBPM[bpmIndex] = rawBPM;
        valuesBPMSum = 0;

        for(int i=0; i<10; i++)
        {
          valuesBPMSum += valuesBPM[i];
        }

        bpmIndex++;
        bpmIndex = bpmIndex % 10;

        if(valuesBPMCount < 10)
          valuesBPMCount++;

        currentBPM = valuesBPMSum / valuesBPMCount;


        currentPulseDetectorState = PULSE_TRACE_DOWN;

        return true;
      }
      break;

    case PULSE_TRACE_DOWN:
    	//HAL_UART_Transmit(&huart5, "TRUE4\n\r", sizeof("TRUE4\n\r"), 100);
      if(sensor_value < prev_sensor_value)
      {
        values_went_down++;
      }


      if(sensor_value < 100)
      {
        currentPulseDetectorState = PULSE_IDLE;
      }
      break;
  }

  prev_sensor_value = sensor_value;
  return false;
}

void balanceIntesities( float redLedDC, float IRLedDC )
{
	uint32_t currentTime = millis();
  if( currentTime - lastREDLedCurrentCheck >= 500)
  {
	  //HAL_UART_Transmit(&huart5, "check sum\n\r", sizeof("check sum\n\r"), 100);
	  if(IRLedDC - redLedDC > 65000 && redLEDCurrent < MAX30100_LED_CURRENT_50MA)
    {
      redLEDCurrent++;
      HAL_UART_Transmit(&huart5, "redLEDCurrent++\n\r", sizeof("redLEDCurrent++\n\r"), 100);
      Max30102_setLedCurrent(RED_LED, redLEDCurrent);
      Max30102_setLedCurrent(IR_LED, IrLedCurrent);
    }
    else if(redLedDC - IRLedDC > 65000 && redLEDCurrent > 0)
    {
      redLEDCurrent--; Max30102_setLedCurrent(RED_LED, redLEDCurrent);
      HAL_UART_Transmit(&huart5, "redLEDCurrent--\n\r", sizeof("redLEDCurrent--\n\r"), 100);
      Max30102_setLedCurrent(IR_LED, IrLedCurrent);
    }

    lastREDLedCurrentCheck = millis();
  }
}


uint32_t millis(void)
{
	return HAL_GetTick();
}
void  Max30102_setLedCurrent(uint8_t led, float currentLevel)
{
	uint8_t value = 0;
	uint8_t ledRegister = 0;

	switch(led){
	case RED_LED: ledRegister = LED_PULSE_AMP_1; break;
	case IR_LED:	ledRegister = LED_PULSE_AMP_2; break;
	}

	// slope derived from MAX30102 DataSheet
	if(currentLevel==0xff)
	{
		value=currentLevel;
	}else{
	value = (uint8_t)(5.0 * currentLevel);
	}

	if( Max30102_writeRegister(ledRegister, value) != HAL_OK){
		return;
	}
	else{

	}
}
void  Max30102_resetFifo(void)
{
	Max30102_writeRegister(FIFO_WRITE_PTR, 0);
	Max30102_writeRegister(FIFO_READ_POINTER, 0);
	Max30102_writeRegister(FIFO_OVF_COUNTER, 0);
}

void print_Sensor_value(float x)
{
    uint8_t buf[500];
    memset(buf,0,500);
    int count=0;
    uint8_t temp;
    int start=0;
    int end=0;
    if(x==0)
    {
    	HAL_UART_Transmit(&huart5, "THE DATA IS 0\n\r",sizeof("THE DATA IS 0\n\r"), 100);
    	return;
    }
    if(x<0)
       {
       	HAL_UART_Transmit(&huart5, "THE DATA IS small than 0\n\r",sizeof("THE DATA IS small than 0\n\r"), 100);
       	return;
       }
    while(x!=0)
        {
            temp=(uint32_t)x%10+'0';
            x=(uint32_t)x/10;
            buf[count]=temp;
            count++;
        }
        buf[count]='\0';
        end=count-1;
        while(start<end)
            {
                temp=buf[end];
                buf[end]=buf[start];
                buf[start]=temp;
                start++;
                end--;

            }
        HAL_UART_Transmit(&huart5, buf, sizeof(buf),100);
        HAL_UART_Transmit(&huart5, space,sizeof(space), 100);

}
void Max30102_resetRegisters(void)
{
	int8_t readStatus;
	uint8_t readResult;


	readStatus = Max30102_readRegister(MODE_CONFIG, &readResult);
	if( readStatus == -1){
		return;
	}

	readResult &= ~(0x01 << 6);
	readResult = readResult | (0x01 << 6);
	if( Max30102_writeRegister(MODE_CONFIG, readResult) != HAL_OK){
		return;
	}
}


void uart_PrintString(char * str)
{
	HAL_UART_Transmit(&huart5, str, strlen(str), HAL_MAX_DELAY);
}

void uart_PrintFloat(float value)
{
	uint8_t buf[12];

	value *= 100;
	sprintf((char*)buf, "%u.%02\r\n",
			(unsigned int)value/100,
			(unsigned int)value % 100);

	HAL_UART_Transmit(&huart5, buf, strlen((char*)buf), HAL_MAX_DELAY);}

void uart_PrintInt(unsigned int value, unsigned char base)
{
	uint8_t buf[12];

	switch(base){
	case 10: sprintf((char*)buf, "%u", value); break;
	case 16: sprintf((char*)buf, "%x", value); break;
	}

	HAL_UART_Transmit(&huart5, buf, strlen((char*)buf), HAL_MAX_DELAY);
}

