/*
 * bq76925.h
 *
 *  Created on: 28 de jul de 2019
 *      Author: ericdrumond
 */

#ifndef INC_BQ76925_H_
#define INC_BQ76925_H_



/* * * * * * * * * * * * * * * * * * * */
/* * * * * Registers addresses * * * * */

#define REG_STATUS 		0x20		/* Read/Write */
#define REG_CELL_CTL 	0x21		/* Read/Write */
#define REG_BAL_CTL 	0x22		/* Read/Write */
#define REG_CONFIG_1 	0x23		/* Read/Write */
#define REG_CONFIG_2	0x24		/* Read/Write */
#define REG_POWER_CTL	0x25		/* Read/Write */
#define REG_CHIP_ID 	0x27		/* Read Only */
#define REG_VREF_CAL 	0x30		/* EEPROM */
#define REG_VC1_CAL 	0x31		/* EEPROM */
#define REG_VC2_CAL 	0x32		/* EEPROM */
#define REG_VC3_CAL 	0x33		/* EEPROM */
#define REG_VC4_CAL 	0x34		/* EEPROM */
#define REG_VC5_CAL 	0x35		/* EEPROM */
#define REG_VC6_CAL 	0x36		/* EEPROM */
#define REG_VC_CAL_EXT1 0x37		/* EEPROM */
#define REG_VC_CAL_EXT2 0x38		/* EEPROM */
#define REG_VREF_CAL_EXT	0x3B	/* EEPROM */
/* End of Registers Addresses*/



/* * * * * * * * * * * * * * * * * * * * */
/* * * * * ADC CHANNELS for STM * * * * */

#define ADCHAN_V_TEMP_1	3	/* ADC_IN3 - Channels for Temperature 1 reading */
#define ADCHAN_V_TEMP_2	4	/* ADC_IN4 - Channels for Temperature 2 reading */
#define ADCHAN_VREF		5	/* ADC_IN5 - Channel for Reference voltage reading */
#define ADCHAN_VCOUT	6	/* ADC_IN6 - Channel for Cell voltage reading */
#define ADCHAN_VIO		7	/* ADC_IN7 - Channel for current reading */
/* End of ADC Channels */

/* * * * * * * * * * * * * * * * * * * * * * * */
/* * * *  Battery Stack data structure  * * * */

typedef struct
{
	uint32_t cellVoltage[6];
	uint32_t cellTemp[2];
	uint16_t current;
	uint32_t stackVoltage;
	uint32_t vRef100;
	uint32_t vRef085;
	uint32_t vRef050;
	uint32_t vSS;
	uint16_t tempTable[71];
} Battery_type;
/* End of Calibration data structure*/

/* * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * Calibration data structure * * * * */

typedef struct
{
	int8_t vcOffset_Corr[6];
	int8_t vcGain_Corr[6];
	int8_t vrefOffset_Corr;
	int8_t vrefGain_Corr;
	int8_t REF_SEL; /* 1.5 V if 0 - 3.0 V if 1*/
} Calibration_type;
/* End of Calibration data structure*/



/* * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * Masks for Calibration Structure * * * * */

#define OFFSET_CORR	0xF0
#define GAIN_CORR	0x0F

#define VC1_OC_4	0x80
#define VC1_GC_4	0x40
#define VC2_OC_4	0x20
#define VC2_GC_4	0x10
#define VC3_OC_4	0x80
#define VC3_GC_4	0x40
#define VC4_OC_4	0x20
#define VC4_GC_4	0x10
#define VC5_OC_4	0x08
#define VC5_GC_4	0x04
#define VC6_OC_4	0x02
#define VC6_GC_4	0x01
#define VREF_OC5	0x04
#define VREF_OC4	0x02
#define VREF_GC4	0x01
/* End of Masks for Calibration Structure */



/* * * * * * * * * * * * * * * * * * */
/* * * * * Register Commands * * * * */


/* Commands for REG_STATUS */
/* Read status of alert, CRC-Error and the power-on reset flag. */
#define STATUS_ALERT	0x04	/* ‘1’ = over-current */
#define STATUS_CRC_ER	0x02	/* ‘1’ = CRC error */
#define STATUS_POR		0x01	/* Set on each power-up and wake-up from sleep */
/* End of commands for REG_STATUS */


/* * *Commands for REG_CELL_CTL * * */
/* Choose what to output on VCOUT pin 15 */
#define VCOUT_VSS		0x00		// VCOUT = VSS
#define VCOUT_VREF05	0x20		// VCOUT = VREF * 0.5
#define VCOUT_VREF085	0x30		// VCOUT = VREF * 0.85
#define VCOUT_VC1		0x10		// VCOUT = VC1
#define VCOUT_VC2		0x11		// VCOUT = VC2
#define VCOUT_VC3		0x12		// VCOUT = VC3
#define VCOUT_VC4		0x13		// VCOUT = VC4
#define VCOUT_VC5		0x14		// VCOUT = VC5
#define VCOUT_VC6		0x15		// VCOUT = VC6
#define VCOUT_VTEMP		0x16		// VCOUT = V_TEMP,INT
#define VCOUT_HI_Z		0x17		// VCOUT = Hi-Z
/* End of commands for REG_CELL_CTL */


/* * * Commands for REG_BAL_CTL * * */
/* Select which cell to start balancing, adjacent balance is not permitted */
/* Default: no cell balancing */
#define BAL_1	0x01
#define BAL_2	0x02
#define BAL_3	0x04
#define BAL_4	0x08
#define BAL_5	0x10
#define BAL_6	0x20
/* End of commands for REG_BAL_CTL */


/* * * Commands for REG_CONFIG_1 (CURRENT) * * */
/* Sets Current comparator threshold, comparator polarity, amplifier calibration, amplifier gain*/
/* Default:	Comparator threshold(0): 25 mV
 * 			I_COMP_POL(0): trips on discharge current (SENSEP > SENSEN)
 * 			I_AMP_CAL(0): current amplifier reports SENSEN with respect to VSS.
 * 			I_GAIN(0): 4 */

/* Standard init for REG_CONFIG_1 is */
#define INIT_REG_CONFIG_1 0x01

/* Bits for toggling */
#define BIT_CONFIG1_IGAIN	0x01
#define BIT_CONFIG1_IAMPCAL	0x04
#define BIT_CONFIG1_ICOMPOL	0x08
#define BIT_CONFIG1_ITRESH1	0x10
#define BIT_CONFIG1_ITRESH2	0x20
#define BIT_CONFIG1_ITRESH3	0x40
#define BIT_CONFIG1_ITRESH4	0x80
/* End of commands for REG_CONFIG_1 */


/* * * Commands for REG_CONFIG_2 * * */
/* Enable CRC comparison on write, reference voltage selection, cell-voltage amplifier gain, VIOUT range */
/* Default:	CRC comparison disabled
 * 			VREF(0): 1.5 V
 * 			VCOUT Gain(0): 0.3
 * 			VIOUT Range(0): 0.25 - 1.25 */

/* Standard init for REG_CONFIG_2 is selecting the voltage reference to be 3.0 V */
#define INIT_REG_CONFIG_2	(BIT_CONFIG2_REFSEL)

/* Bits for toggling */
#define BIT_CONFIG2_REFSEL	0x01
#define BIT_CONFIG2_CRCEN	0x80
/* End of commands for REG_CONFIG_2 */


/* * * Commands for REG_POWER_CTL * * */
/* Settings to save power, such as sleep mode, enable/disable features */
/* Default: SLEEP(0): no sleep
 * 			SLEEP_DIS(0): sleep mode enabled
 * 			Not used(0)
 * 			I_COMP_EN(0): current comparator disabled
 * 			I_AMP_EN(0): current amplifier disabled
 * 			VC_AMP_EN(0): cell amplifier disabled
 * 			VTB_EN(0): thermistor bias disabled
 * 			REF_EN(0): voltage reference disabled */

/* Standard init for REG_POWER_CTL is disabling sleep mode only */
#define INIT_REG_POWER_CTL	(BIT_POWERCTL_SLPDIS+BIT_POWERCTL_IAMP)

/* Bits for toggling */
#define BIT_POWERCTL_REFEN	0x01
#define BIT_POWERCTL_VTEMP	0x02
#define BIT_POWERCTL_VCAMP	0x04
#define BIT_POWERCTL_IAMP	0x08
#define BIT_POWERCTL_ICOMP	0x10
#define BIT_POWERCTL_SLPDIS	0x40
#define BIT_POWERCTL_SLEEP	0x80
/* End of commands for REG_POWER_CTL */



/* * * * * * * * * * * * * * * * * * */
/* * * * * Function defines * * * * */

/* General Functions*/
void bq_Init(Calibration_type *cal, Battery_type *bat, ADC_TypeDef *ADCx);
void batStruct_Init(Battery_type *bat);
void writeRegister(uint8_t address, uint8_t command);
void readRegister(uint8_t address, uint8_t *data);
void toggleBits(uint8_t address, uint8_t numBits, ...);
/* End of General Functions*/

/* Voltage reading Functions */
void request_VCout(uint8_t command);
void voltage_Measuring(ADC_TypeDef *ADCx, Battery_type *bat, Calibration_type cal);
uint32_t corrected_Voltage(uint8_t cellNumber, uint16_t ADC_Count, uint16_t vRef, Calibration_type cal);
void getCalParamVCOUT(Calibration_type *cal);
/* End of Voltage reading Functions */

/* Temperature reading Functions */
void temperature_Measuring(ADC_TypeDef *ADCx, Battery_type *bat);
uint32_t corrected_Temperature(Battery_type *bat, uint32_t ADC_Count);
/* End of Temperature reading Functions*/

/* Current reading Functions */
void current_Measuring(ADC_TypeDef *ADCx, Battery_type *bat);
uint16_t corrected_Current(uint32_t sensen_ADC, uint32_t sensep_ADC);
/* End of Current reading Functions */

/* ADC Functions */
uint8_t calibrate_ADC(ADC_TypeDef *ADCx);
uint16_t get_ADC12bits_channel(ADC_TypeDef *ADCx, uint8_t channel);
void calibrateVRef(ADC_TypeDef *ADCx, Battery_type *bat, Calibration_type *cal);
/* End of ADC Functions */

/* End of Function defines */
void createTempTable(Battery_type *bat);

#endif
