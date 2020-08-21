/*
 * bq76925.c
 *
 *  Created on: 28 de jul de 2019
 *      Author: ericdrumond
 */

#include "adc.h"
#include "stdarg.h"
#include "stdio.h"
#include "stm32l0xx.h"
#include "stm32l0xx_ll_tim.h"
#include "stm32l0xx_ll_iwdg.h"
#include "stm32l0xx_it.h"
#include "gpio.h"
#include "stm32l0xx_ll_exti.h"
#include "base_lib.h"
#include "bq76925.h"
#include "defines.h"
#include "stdlib.h"


int32_t vgain_ad=10000, voff_ad=0;
extern uint16_t var_count_sum;
extern uint8_t flag_count_clb;
/**
 * @brief  Initialize BQ76925 registers CONFIG_1, CONFIG_2 and POWER_CTL and gets Calibration values.
 * @param 	*cal Calibration_type Instance
 * @note	The values must be edited in the bq76925.h library on defines
 * #define INIT_REG_CONFIG1
 * #define INIT_REG_CONFIG2
 * #define INIT_REG_POWER_CTL
 * @retval None
 */
void bq_Init(Calibration_type *cal, Battery_type *bat, ADC_TypeDef *ADCx){
	writeRegister(REG_CONFIG_1, INIT_REG_CONFIG_1);
	writeRegister(REG_CONFIG_2, INIT_REG_CONFIG_2);
	writeRegister(REG_POWER_CTL, INIT_REG_POWER_CTL);
	getCalParamVCOUT(cal);
	batStruct_Init(bat);
	calibrateVRef(ADCx, bat, cal);
}


/**
 * @brief  Initialize Battery structure with Zeroes.
 * @param 	*bat Battery_type Instance
 * @retval None
 */
void batStruct_Init(Battery_type *bat){
	for(int i = 0; i < 6; i++){
		bat->cellVoltage[i] = 0;
	}
	bat->cellTemp[0] = 0xbb;
	bat->cellTemp[1] = 0xaa;
	createTempTable(bat);
}

void calibrateVRef(ADC_TypeDef *ADCx, Battery_type *bat,Calibration_type *cal ){

	toggleBits(REG_POWER_CTL, 2, BIT_POWERCTL_REFEN, BIT_POWERCTL_VCAMP);
	delay_ms(10);
	int32_t vdif_ad=0, i, v1, v2, aux_vref;
	bat->vRef100 =0;
	bat->vRef085 =0;
	bat->vRef050 =0;
	bat->vSS =0;

	/* Get vRef (3.00V) */
	for(i=0;i<256;i++){
		bat->vRef100 = bat->vRef100+ get_ADC12bits_channel(ADCx, ADCHAN_VREF);
		LL_IWDG_ReloadCounter(IWDG);
	}
	bat->vRef100= bat->vRef100 >>8;

	/* Get (0.85*vRef = 2.55V) from Vcout  */
	request_VCout(VCOUT_VREF085);
	delay_ms(2);
	for(i=0;i<256;i++){
		bat->vRef085 = bat->vRef085+ get_ADC12bits_channel(ADCx, ADCHAN_VCOUT);
		LL_IWDG_ReloadCounter(IWDG);
	}
	bat->vRef085= bat->vRef085 >>8;

	/* Get (0.50*vRef = 1.5V) from Vcout */
	request_VCout(VCOUT_VREF05);
	delay_ms(2);
	for(i=0;i<256;i++){
		bat->vRef050 = bat->vRef050+ get_ADC12bits_channel(ADCx, ADCHAN_VCOUT);
		LL_IWDG_ReloadCounter(IWDG);
	}
	bat->vRef050= bat->vRef050 >>8;

	/* Get Vss */
	request_VCout(VCOUT_VSS);
	delay_ms(2);
	for(i=0;i<256;i++){
		bat->vSS = bat->vSS+ get_ADC12bits_channel(ADCx, ADCHAN_VCOUT);
		LL_IWDG_ReloadCounter(IWDG);
	}
	bat->vSS= bat->vSS >>8;

	bat->vSS = get_ADC12bits_channel(ADCx, ADCHAN_VCOUT);

	toggleBits(REG_POWER_CTL, 2, BIT_POWERCTL_REFEN, BIT_POWERCTL_VCAMP);

//	vdif_ad = bat->vRef085 - bat->vRef050; //metodo 1: 0,25% de erro
//	vgain_ad = (1302 * 10000) /vdif_ad;
//	voff_ad = 1861*10000 - (vgain_ad * bat->vRef050);

	if(cal->vrefGain_Corr > 0)
	{
		aux_vref=(1000+cal->vrefGain_Corr);
	}
	else
	{
		aux_vref=(1000 - (uint32_t)(cal->vrefGain_Corr & 0b1111));
	}
	if(cal->vrefOffset_Corr > 0)
	{
		bat->vRef100 = (bat->vRef100*aux_vref) + (((uint32_t)(cal->vrefOffset_Corr)*5)/4);
	}
	else
	{
		bat->vRef100 = (bat->vRef100*aux_vref) - (((uint32_t)(cal->vrefOffset_Corr &0b11111)*5)/4);
	}

	v1=(bat->vRef100 * 5);
	v2=(bat->vRef100 * 85)/10;
	vdif_ad = bat->vRef085 - bat->vRef050;
	vgain_ad = (v2-v1) /vdif_ad;
	voff_ad = v1 - (vgain_ad * bat->vRef050);
}

/**
 * @brief  Send single command to BQ76925 register.
 * @param  address This is the i2c register address
 * 		@arg @ref REG_STATUS
 * 		@arg @ref REG_CELL_CTL
 * 		@arg @ref REG_BAL_CTL
 * 		@arg @ref REG_CONFIG_1
 * 		@arg @ref REG_CONFIG_2
 * 		@arg @ref REG_POWER_CTL
 * @param  command This is the command/data to write on target register
 * @retval None
 */
void writeRegister(uint8_t address, uint8_t command){

	LL_I2C_HandleTransfer(I2C1, (address << 1), LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

	while(!LL_I2C_IsActiveFlag_STOP(I2C1)){
		/* Indicate the status of Transmit data register empty flag */
		if(LL_I2C_IsActiveFlag_TXIS(I2C1)){
			LL_I2C_TransmitData8(I2C1, 0b11111111 & command); /* Byte to send */
		}
	}
	LL_I2C_ClearFlag_STOP(I2C1);
}



/**
 * @brief  Read single register from BQ76925.
 * @param  address This is the i2c register address
 * 		@arg @ref REG_STATUS
 * 		@arg @ref REG_CELL_CTL
 * 		@arg @ref REG_BAL_CTL
 * 		@arg @ref REG_CONFIG_1
 * 		@arg @ref REG_CONFIG_2
 * 		@arg @ref REG_POWER_CTL
 * 		@arg @ref REG_CHIP_ID
 * 		@arg @ref REG_VREF_CAL
 * 		@arg @ref REG_VC1_CAL
 * 		@arg @ref REG_VC2_CAL
 * 		@arg @ref REG_VC3_CAL
 * 		@arg @ref REG_VC4_CAL
 * 		@arg @ref REG_VC5_CAL
 * 		@arg @ref REG_VC6_CAL
 * 		@arg @ref REG_VC_CAL_EXT1
 * 		@arg @ref REG_VC_CAL_EXT2
 * 		@arg @ref REG_VREF_CAL_EXT
 * @param  *data This is the data uint8_t address that will receive the data byte
 * @retval None
 */
void readRegister(uint8_t address, uint8_t *data){

	LL_I2C_HandleTransfer(I2C1, (address << 1), LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);

	while(!LL_I2C_IsActiveFlag_STOP(I2C1)){
		/* Receive data (RXNE flag raised) */
		/* Check RXNE flag value in ISR register */
		if(LL_I2C_IsActiveFlag_RXNE(I2C1)){
			/* Read character in Receive Data register.
			      RXNE flag is cleared by reading data in RXDR register */
			*data = LL_I2C_ReceiveData8(I2C1); /* Byte to send */
		}
	}
	LL_I2C_ClearFlag_STOP(I2C1);
}



/**
 * @brief  Toggle single or multiple bits on any BQ76925 register.
 * @param  address This is the i2c register address
 * 		@arg @ref REG_STATUS
 * 		@arg @ref REG_CELL_CTL
 * 		@arg @ref REG_BAL_CTL
 * 		@arg @ref REG_CONFIG_1
 * 		@arg @ref REG_CONFIG_2
 * 		@arg @ref REG_POWER_CTL
 * @param  numBits This is the number of bits that will be changed
 * @param	"..." The bit position to toggle on wanted register. E.g, the LSB on register will be 0x01
 * @retval None
 */
void toggleBits(uint8_t address, uint8_t numBits, ...){
	uint8_t currentRegValue;
	uint8_t currentBit;
	va_list valist;
	va_start(valist, numBits);

	/* For each of the "numBits" bits */
	for(int i = 0; i < numBits; i++){
		currentBit = (uint8_t)(0xFF & va_arg(valist, int));
		readRegister(address, &currentRegValue);
		if((currentRegValue & currentBit) == currentBit){
			/* The wanted bit is currently '1', then reset it */
			writeRegister(address, CLEAR_BIT(currentRegValue, currentBit));
		}
		else{
			/* The wanted bit is currently '0', then set it */
			writeRegister(address, SET_BIT(currentRegValue, currentBit));
		}
	}
	va_end(valist);
}



/**
 * @brief  Send single command to BQ76925 requesting change on VCOUT pin.
 * @param  command This is the i2c data written to the REG_CELL_CTL register
 * 		@arg @ref VCOUT_VSS
 * 		@arg @ref VCOUT_VREF05
 * 		@arg @ref VCOUT_VREF085
 * 		@arg @ref VCOUT_VC1
 * 		@arg @ref VCOUT_VC2
 * 		@arg @ref VCOUT_VC3
 * 		@arg @ref VCOUT_VC4
 * 		@arg @ref VCOUT_VC5
 * 		@arg @ref VCOUT_VC6
 * 		@arg @ref VCOUT_VTEMP
 * 		@arg @ref VCOUT_HI_Z
 * @retval None
 */
void request_VCout(uint8_t command){
	writeRegister(REG_CELL_CTL, command);
}



/**
 * @brief  Read all 6 cell voltages of BQ76925 through ADC.
 * @param 	*ADCx Is the ADC_TypeDef used for this channel
 * @param  voltage_cell - Array of cell voltages address.
 * @param	cal - calibration struct address.
 * @retval None
 */
void voltage_Measuring(ADC_TypeDef *ADCx, Battery_type *bat, Calibration_type cal){
	uint8_t i;
	uint16_t ADC_Count = 0;
	uint16_t stackTotal = 0;
	volatile uint16_t vRef;
	bat->stackVoltage = 0;

	/* Sets the commands vector to read each of the 6 battery cells */
	uint8_t commands[6] = {VCOUT_VC1, VCOUT_VC2, VCOUT_VC3, VCOUT_VC4, VCOUT_VC5, VCOUT_VC6};

	/* Enables VC Amplifier on Register Power CTL to read VCOUT pin*/
	toggleBits(REG_POWER_CTL, 2, BIT_POWERCTL_VCAMP, BIT_POWERCTL_REFEN);
	delay_ms(1);
	toggleBits(REG_POWER_CTL, 1, BIT_POWERCTL_VTEMP);
	delay_ms(10);// delay necessario para que a primeira leitura nao seja comprometida


	/* For each requested VCout cell voltage do: */
	for(i = 0; i < 6; i++){
		LL_IWDG_ReloadCounter(IWDG);
		request_VCout(commands[i]);
		delay_ms(2); //arrumar

		/* Get ADC_Count for current cell */
		/* VCOU_CHANNEL is defined in the bq76925.h file */
		//LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
		ADC_Count = get_ADC12bits_channel(ADCx, ADCHAN_VCOUT);
		LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);


		/* Sets the voltage reference from BQ76925 */
		vRef = get_ADC12bits_channel(ADCx, ADCHAN_VREF);

		/* Get corrected real voltage through the VCn equation */
		bat->cellVoltage[i] = (uint16_t) corrected_Voltage(i, ADC_Count, vRef, cal);
		stackTotal += bat->cellVoltage[i];
	}
	//LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);
	/* Disable the VC Amplifier on Register Power CTL to save power*/
	toggleBits(REG_POWER_CTL, 2, BIT_POWERCTL_VCAMP, BIT_POWERCTL_REFEN);

	bat->stackVoltage = stackTotal;
	LL_IWDG_ReloadCounter(IWDG);

}


/**
 * @brief  Use the bq76925 equation to get the actual cell voltage (mV) after getting the ADC value.
 * @param  cellNumber Number of current cell (1, .., 6)
 * @param	ADC_Count Is the ADC conversion data for current cell
 * @param	*cal Is the address of the Calibration_type structure
 * @retval Corrected voltage value in milivolts (mV).
 */
uint32_t corrected_Voltage(uint8_t cellNumber, uint16_t ADC_Count, uint16_t vRef, Calibration_type cal){
	uint32_t vref_Nominal = 0;
	uint32_t correct_ADC_Count = 0;
	uint32_t vcout = 0;
	uint32_t vcn = 0;
	int32_t gc_vcout = 0;
	int32_t oc_vcout = 0;
	uint32_t gc_vref = 0;
	uint32_t g_vcout = 0;
	int32_t aux_adc;
	int32_t aux_vref;

	//	vcn=ADC_Count;

	if(cal.vrefGain_Corr > 0)
	{
		aux_vref=(1000+cal.vrefGain_Corr);
	}
	else
	{
		aux_vref=(1000 - (uint32_t)(cal.vrefGain_Corr & 0b1111));
	}
	if(cal.vrefOffset_Corr > 0)
	{
		aux_vref = (vRef*aux_vref) + (((uint32_t)(cal.vrefOffset_Corr)*5)/4);

	}
	else
	{
		aux_vref = (vRef*aux_vref) - (((uint32_t)(cal.vrefOffset_Corr &0b11111)*5)/4);
	}

	vRef=(uint16_t) (aux_vref/1000);


	aux_adc = (int32_t) ADC_Count;
	aux_adc = ((((((aux_adc*10000) + voff_ad)/10000) * vgain_ad)*3)/vRef)/10;
	ADC_Count = (uint16_t) (aux_adc & 0xffff);
	vcout=(uint32_t) aux_adc;
	gc_vref = 1; //(1000 + (cal->vrefGain_Corr)) + (cal->vrefOffset_Corr / (vref_Nominal/1000);
	oc_vcout = cal.vcOffset_Corr[cellNumber]; // mV
	gc_vcout = cal.vcGain_Corr[cellNumber]; // 0.1%
	g_vcout = (cal.REF_SEL == 1) ? 600 : 300;
	vcn = (((vcout * gc_vref + oc_vcout) * ((1000 + gc_vcout)*1000)) / g_vcout)/1000;

	return (uint16_t)vcn;
}



/**
 * @brief  Read all Calibration parameters from BQ76925.
 * @param  *cal Calibration_Type Instance
 * @retval None
 */
void getCalParamVCOUT(Calibration_type *cal){
	uint8_t vcx_Cal[6];
	uint8_t vcx_Cal_Ext[2];
	uint8_t vref_Cal;
	uint8_t vref_Cal_Ext;
	uint8_t vcx_OC_4[6] = {VC1_OC_4, VC2_OC_4, VC3_OC_4, VC4_OC_4, VC5_OC_4, VC6_OC_4};
	uint8_t vcx_GC_4[6] = {VC1_GC_4, VC2_GC_4, VC3_GC_4, VC4_GC_4, VC5_GC_4, VC6_GC_4};
	uint8_t vcx_Cal_Regs[6] = {REG_VC1_CAL, REG_VC2_CAL, REG_VC3_CAL, REG_VC4_CAL, REG_VC5_CAL, REG_VC6_CAL};
	uint8_t vcx_Cal_Ext_Regs[2] = {REG_VC_CAL_EXT1, REG_VC_CAL_EXT2};
	uint8_t aux;
	uint8_t refSel_Aux;

	readRegister(REG_VREF_CAL, &vref_Cal);
	readRegister(REG_VREF_CAL_EXT, &vref_Cal_Ext);

	for(int i = 0; i < 6; i++){
		readRegister(vcx_Cal_Regs[i], &vcx_Cal[i]);
		if(i < 2){
			readRegister(vcx_Cal_Ext_Regs[i], &vcx_Cal_Ext[i]);
		}
	}

	/* 6-bit signed 2�s complement number in the range �32 to +31 with a value of 1 mV per LSB */
	cal->vrefOffset_Corr = ((vref_Cal & OFFSET_CORR) >> 4) + (((vref_Cal_Ext & VREF_OC4) == VREF_OC4) << 4) + (((vref_Cal_Ext & VREF_OC5) == VREF_OC5) << 5);
	/* If the 6th bit is 1, extend sign */
	if((cal->vrefOffset_Corr & 0b100000) == 0b100000){
		cal->vrefOffset_Corr = cal->vrefOffset_Corr + 0b11000000;
	}

	/*  5-bit signed 2�s complement number in the range �16 to +15 with a value of 0.1% per lsb */
	cal->vrefGain_Corr = (vref_Cal & GAIN_CORR) + (((vref_Cal_Ext & VREF_GC4) == VREF_GC4) << 4);
	/* If the 5th bit is 1, extend sign */
	if((cal->vrefGain_Corr & 0b10000) == 0b10000){
		cal->vrefGain_Corr = cal->vrefGain_Corr + 0b11100000;
	}

	for(int i = 0; i < 6; i++){
		aux = (i < 2) ? 0 : 1;
		/* 5-bit signed 2�s complement number in the range �16 to +15 with a value of 1 mV per LSB */
		cal->vcGain_Corr[i] = (vcx_Cal[i] & GAIN_CORR) + (((vcx_Cal_Ext[aux] & vcx_GC_4[i]) == vcx_GC_4[i]) << 4);
		/* If the 5th bit is 1, extend sign */
		if((cal->vcGain_Corr[i] & 0b10000) == 0b10000){
			cal->vcGain_Corr[i] = cal->vcGain_Corr[i] + 0b11100000;
		}


		/* 5-bit signed 2�s complement number in the range �16 to +15 with a value of 0.1% per LSB */
		cal->vcOffset_Corr[i] =	((vcx_Cal[i] & OFFSET_CORR) >> 4) + (((vcx_Cal_Ext[aux] & vcx_OC_4[i]) == vcx_OC_4[i]) << 4);
		/* If the 5th bit is 1, extend sign */
		if((cal->vcOffset_Corr[i] & 0b10000) == 0b10000){
			cal->vcOffset_Corr[i] = cal->vcOffset_Corr[i] + 0b11100000;
		}
	}

	readRegister(REG_CONFIG_2, &refSel_Aux);
	cal->REF_SEL = ((refSel_Aux & BIT_CONFIG2_REFSEL) == BIT_CONFIG2_REFSEL) ? 1 : 0;
}



/**
 * @brief  Read all 2 cell temperatures through ADC.
 * @param 	*ADCx Is the ADC_TypeDef used for this channel
 * @param  temperature_Sensors - Array of temperatures.
 * @retval None
 */
void temperature_Measuring(ADC_TypeDef *ADCx, Battery_type *bat){
	uint8_t i;
	uint32_t ADC_Count = 0;

	/* Sets the ADC Channels for each thermistor */
	/* ADCHAN_V_TEMP is defined in the bq76925.h file */
	uint8_t ADC_Channel[2] = {ADCHAN_V_TEMP_1, ADCHAN_V_TEMP_2};

	/* Enables VTEMP on Register Power CTL to bias the thermistors circuit */
	//toggleBits(REG_POWER_CTL, 1, BIT_POWERCTL_VTEMP);
	delay_ms(10);

	/* For each thermistor voltage do: */
	for(i = 0; i < 2; i++){

		/* Get ADC_Count for current thermistor */
		//bat->cellTemp[i]= get_ADC12bits_channel(ADCx, ADC_Channel[i]);
		ADC_Count = get_ADC12bits_channel(ADCx, ADC_Channel[i]);

		/* Get corrected real temperature */
		bat->cellTemp[i] = corrected_Temperature(bat, ADC_Count);
	}

	/* Disable the VTEMP Amplifier on Register Power CTL to save power*/
	toggleBits(REG_POWER_CTL, 1, BIT_POWERCTL_VTEMP);
	LL_IWDG_ReloadCounter(IWDG);
}



/**
 * @brief  Get the actual temperature (�C) after getting the ADC value.
 * @param  *bat Battery_type instance
 * @param	ADC_Count Is the ADC conversion data for current thermistor
 * @retval Corrected temperature value in degrees celsius (�C).
 */
#pragma GCC push_options
#pragma GCC optimize (0)
uint32_t corrected_Temperature(Battery_type *bat, uint32_t ADC_Count){
	uint16_t res = 20;
	uint16_t vtb = 3300;
	uint32_t vTerm = (ADC_Count*3300)/4096;
	uint32_t vRes = (vTerm*1000) / ((vtb - vTerm) / res);
	uint16_t rTerm = (uint16_t) vRes;

	//return vRes;

	uint8_t searchIndex = 35, beginIndex = 0, endIndex = 70;
	uint16_t binSearch = bat->tempTable[searchIndex];
	uint16_t tolerance = rTerm/100;
	volatile uint8_t tries = 1;


	/* Makes a binary search on the Temperature x Resistance array (bat->tempTable) */
	while(((rTerm > (binSearch + tolerance)) || (rTerm < (binSearch - tolerance))) && (tries < 9)){

		if(rTerm < binSearch){
			beginIndex = searchIndex;
			searchIndex = ((endIndex - beginIndex) / 2) + beginIndex;
		}
		else{
			endIndex = searchIndex;
			searchIndex = ((endIndex - beginIndex) / 2) + beginIndex;
		}
		binSearch = bat->tempTable[searchIndex];
		tolerance = binSearch/100;
		tries++;
	}

	//uint32_t temperature = ADC_Count;
	return (uint32_t) searchIndex;
}
#pragma GCC pop_options


/**
 * @brief  Read current value through ADC.
 * @param 	*ADCx Is the ADC_TypeDef used for this channel
 * @param	*bat Battery_type instance
 * @retval None
 */
void current_Measuring(ADC_TypeDef *ADCx, Battery_type *bat){
	uint32_t sensen_ADC = 0, sensep_ADC = 0;

	/* Enables VTEMP on Register Power CTL to bias the current measurement circuit */
//	toggleBits(REG_POWER_CTL, 1, BIT_POWERCTL_IAMP);

	/* BIT_CONFIG1_IAMPCAL = 0 */
//	toggleBits(REG_CONFIG_1, 1, BIT_CONFIG1_IAMPCAL);
	writeRegister(REG_CONFIG_1, INIT_REG_CONFIG_1 | BIT_CONFIG1_IAMPCAL);

	delay_ms(2);


	/* Get sensen_ADC with respect to VSS */
	/* BIT_CONFIG1_IAMPCAL = 0 */
	sensen_ADC = get_ADC12bits_channel(ADCx, ADCHAN_VIO);

	/* Get sensen_ADC with respect to VSS */
	/* BIT_CONFIG1_IAMPCAL = 1 */
//	toggleBits(REG_CONFIG_1, 1, BIT_CONFIG1_IAMPCAL);
	writeRegister(REG_CONFIG_1, INIT_REG_CONFIG_1);
	delay_ms(10);
	sensep_ADC = get_ADC12bits_channel(ADCx, ADCHAN_VIO);
	writeRegister(REG_CONFIG_1, INIT_REG_CONFIG_1 | BIT_CONFIG1_IAMPCAL);
	/* Get corrected real current Measurement */
//	bat->current = (uint16_t) corrected_Current(sensen_ADC, sensep_ADC);
	if(flag_count_clb==0)
	{
	bat->current = corrected_Current(sensen_ADC, sensep_ADC);
	}
	else{
		var_count_sum= corrected_Current(sensen_ADC, sensep_ADC);
	}

	/* Disable the VIO Amplifier on Register Power CTL to save power*/
//	toggleBits(REG_POWER_CTL, 1, BIT_POWERCTL_IAMP);
//	delay_ms(50);
}



/**
 * @brief  Get the actual current (A) after getting the ADC value.
 * @param	ADC_Count Is the ADC conversion data for current in VIO
 * @retval Corrected current value in amperes (A).
 */
uint16_t corrected_Current(uint32_t sensen_ADC, uint32_t sensep_ADC){
/*	uint8_t g_vioutAux;
	uint8_t g_viout = 0;
	int32_t current = 0;
	int32_t vsense = 0;
	vsense = 5000 + ((int32_t)(sensep_ADC) - (int32_t)(sensen_ADC));
*/
	uint32_t vsense = 0;
	vsense = sensep_ADC + 5000;
	vsense = vsense - sensen_ADC;
	return (uint16_t) vsense;

/*
	readRegister(REG_CONFIG_1, &g_vioutAux);
	g_viout = ((g_vioutAux & BIT_CONFIG1_IGAIN) == BIT_CONFIG1_IGAIN) ? 8 : 4;
	vsense = (((int32_t)(sensep_ADC) - (int32_t)(sensen_ADC))*100) / (int8_t)g_viout;
	vsense = (vsense * 330000) / 4096;
	current = (vsense / R_SHUNT); //[mV]/[mOhm] = [A]
	return abs(current);
*/
}

/**
 * @brief  Requests the ADC Calibration for current Vdd, if different of 3.00V
 * @param  *ADCx Is the ADC_TypeDef structure
 * @retval 1 if calibration was successful, 0 if not. ADC must be disabled to do this calibration.
 */
uint8_t calibrate_ADC(ADC_TypeDef *ADCx){
	/* Makes sure to remove all channels from the conversion queue */
	for(int i = 0; i < 19; i++){
		LL_ADC_REG_SetSequencerChRem(ADCx, __LL_ADC_DECIMAL_NB_TO_CHANNEL(i));
	}

	/* Confirms that the ADC is DISABLED */
	if(LL_ADC_IsEnabled(ADCx) == 0){

		/* Starts Calibration */
		LL_ADC_StartCalibration(ADCx);

		/* Waits for end of calibration */
		while(LL_ADC_IsCalibrationOnGoing(ADCx));

		LL_ADC_ClearFlag_EOCAL(ADCx);

		return 1;
	}
	return 0;
}



/**
 * @brief  Gets the ADC conversion value for given ADC channel.
 * @param  channel Is the STM ADC_IN_Channel
 * 	@arg @ref ADCHAN_VREF
 * 	@arg @ref ADCHAN_VCOUT
 * 	@arg @ref ADCHAN_V_TEMP_1
 * 	@arg @ref ADCHAN_V_TEMP_2
 * 	@arg @ref ADCHAN_VIO
 * @retval measurement Is the ADC count between 0 and 4095 (12 bits).
 */

//#pragma GCC push_options
//#pragma GCC optimize (0)
uint16_t get_ADC12bits_channel(ADC_TypeDef *ADCx, uint8_t channel){

	uint32_t measurement=0, i;

	/* Add ADC channel to conversion sequence */
	LL_ADC_REG_SetSequencerChAdd(ADCx, __LL_ADC_DECIMAL_NB_TO_CHANNEL(channel));


	/* Clears the ADC Ready flag */
	LL_ADC_ClearFlag_ADRDY(ADCx);

	/* Enables ADC peripheral */
	LL_ADC_Enable(ADCx);
	/* Waits for the ADC to be ready */
	while (LL_ADC_IsActiveFlag_ADRDY(ADCx) == 0)
	{
		/* Timeout implementation pending */

	}
	/* Start ADC Conversion */
	LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
	for(i=0;i<16;i++)
	{
		LL_ADC_REG_StartConversion(ADCx);

		/* Waits for End-of-Conversion flag to be active */
		while ((ADCx->ISR & ADC_ISR_EOC) == 0)
		{
			/* Timeout implementation pending */
		}

		/* Stops the ADC Conversion if in continuous mode*/
		if(LL_ADC_REG_GetContinuousMode(ADCx) == LL_ADC_REG_CONV_CONTINUOUS){
			//LL_ADC_REG_StopConversion(ADCx);
		}

		/* Clears End-of-Conversion flag */
		LL_ADC_ClearFlag_EOC(ADCx);

		/* Clears End-of-Sequence Flag */
		LL_ADC_ClearFlag_EOS(ADCx);

		/* Clears End-of-Sampling phase Flag */
		LL_ADC_ClearFlag_EOSMP(ADCx);

		/* Retrieve ADC conversion from data register  */
		measurement= measurement + LL_ADC_REG_ReadConversionData12(ADCx);
	}
	measurement = measurement >>4;
	LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);
	/* ensure that no conversion is ongoing */
	if ((ADCx->CR & ADC_CR_ADSTART) != 0)
	{
		/* If there is ongoing conversion, stop it */
		ADCx->CR |= ADC_CR_ADSTP;
	}
	/* Wait until ADSTP is reset by hardware i.e. conversion is stopped */
	while ((ADCx->CR & ADC_CR_ADSTP) != 0) /* (3) */
	{
		/* Timeout implementation pending */
	}
	/* Disables ADC peripheral */
	LL_ADC_Disable(ADCx);

	/* Wait until the ADC is fully disabled */
	while ((ADC1->CR & ADC_CR_ADEN) != 0)
	{
		/* Timeout implementation pending */
	}
	/* Remove ADC channel from conversion sequence */
	LL_ADC_REG_SetSequencerChRem(ADCx, __LL_ADC_DECIMAL_NB_TO_CHANNEL(channel));
	//LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
	return measurement;

}
//#pragma GCC pop_options

void createTempTable(Battery_type *bat){
	/* https://cdn-shop.adafruit.com/datasheets/103_3950_lookuptable.pdf */
	bat->tempTable[0] = 31770;
	bat->tempTable[1] = 30250;
	bat->tempTable[2] = 28820;
	bat->tempTable[3] = 27450;
	bat->tempTable[4] = 26160;
	bat->tempTable[5] = 24940;
	bat->tempTable[6] = 23770;
	bat->tempTable[7] = 22670;
	bat->tempTable[8] = 21620;
	bat->tempTable[9] = 20630;
	bat->tempTable[10] = 19680;
	bat->tempTable[11] = 18780;
	bat->tempTable[12] = 17930;
	bat->tempTable[13] = 17120;
	bat->tempTable[14] = 16350;
	bat->tempTable[15] = 15620;
	bat->tempTable[16] = 14930;
	bat->tempTable[17] = 14260;
	bat->tempTable[18] = 13630;
	bat->tempTable[19] = 13040;
	bat->tempTable[20] = 12470;
	bat->tempTable[21] = 11920;
	bat->tempTable[22] = 11410;
	bat->tempTable[23] = 10910;
	bat->tempTable[24] = 10450;
	bat->tempTable[25] = 10000;
	bat->tempTable[26] = 9575;
	bat->tempTable[27] = 9170;
	bat->tempTable[28] = 8784;
	bat->tempTable[29] = 8416;
	bat->tempTable[30] = 8064;
	bat->tempTable[31] = 7730;
	bat->tempTable[32] = 7410;
	bat->tempTable[33] = 7106;
	bat->tempTable[34] = 6815;
	bat->tempTable[35] = 6538;
	bat->tempTable[36] = 6273;
	bat->tempTable[37] = 6020;
	bat->tempTable[38] = 5778;
	bat->tempTable[39] = 5548;
	bat->tempTable[40] = 5327;
	bat->tempTable[41] = 5117;
	bat->tempTable[42] = 4915;
	bat->tempTable[43] = 4723;
	bat->tempTable[44] = 4539;
	bat->tempTable[45] = 4363;
	bat->tempTable[46] = 4195;
	bat->tempTable[47] = 4034;
	bat->tempTable[48] = 3880;
	bat->tempTable[49] = 3733;
	bat->tempTable[50] = 3592;
	bat->tempTable[51] = 3457;
	bat->tempTable[52] = 3328;
	bat->tempTable[53] = 3204;
	bat->tempTable[54] = 3086;
	bat->tempTable[55] = 2972;
	bat->tempTable[56] = 2863;
	bat->tempTable[57] = 2759;
	bat->tempTable[58] = 2659;
	bat->tempTable[59] = 2564;
	bat->tempTable[60] = 2472;
	bat->tempTable[61] = 2384;
	bat->tempTable[62] = 2299;
	bat->tempTable[63] = 2218;
	bat->tempTable[64] = 2141;
	bat->tempTable[65] = 2066;
	bat->tempTable[66] = 1994;
	bat->tempTable[67] = 1926;
	bat->tempTable[68] = 1860;
	bat->tempTable[69] = 1796;
	bat->tempTable[70] = 1735;
}
