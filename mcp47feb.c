/**
 * @file 	mcp47feb.c
 * @author 	Ed Holmes (eholmes@lablogic.com)
 * @brief	Library to control MCP47FEB I2C Device based on STM32 HAL.
 * 			Depends on STM32 I2C HAL (pulled by "i2c.h" or stm32xxxx_hal_i2c.h".
 * 			Uses EXIT_FAILURE (1) and EXIT_SUCCESS (0) from stdlib.h as return values.
 * 			Tested on MCP47FEB22.
 * @date 	2021-04-08
 */

#include "mcp47feb.h"
#include "stdio.h"
#include "i2c.h"
#include "stdlib.h"


#define word(x,y) 	((x << 8) | y)
#define lowByte(x) 	(x & 0x0F)


#define BASE_ADDR 		0x60
// REG ADDRESSES ALREADY << 3
/* Register Addresses */
#define RESET_REG 		0x06
#define WAKE_REG 		0x0A
#define UPDATE_REG 		0x08 	//Unused
#define GENERALCALL 	0x00
#define READ 			0x06
#define WRITE 			0x00
#define DAC0_REG 		0x00
#define DAC1_REG 		0x01
#define VREF_REG 		0x08
#define PD_REG 			0x09
#define GAIN_REG 		0x0A
#define WL_REG 			0x0B
#define DAC0_EP_REG		0x10 
#define DAC1_EP_REG 	0x11
#define VREF_EP_REG 	0x18
#define PD_EP_REG 		0x19
#define GAIN_EP_REG		0x1A 
/* Command to lock/unlock SALCK. Datasheet Fig 7-14 */
#define SALCK			0xD0
/* SALCK COMMMAND BITS */
#define UNLOCK_SALCK 	0x02
#define LOCK_SALCK 	 	0x04


static uint8_t readBuffer[5];


static void _ReadEpAddr(MCP47FEB_TypeDef *dac, uint8_t REG, uint8_t buffer[5]);
static void _ReadAddr(MCP47FEB_TypeDef *dac, uint8_t REG, uint8_t buffer[5]);
static void _FastWrite(MCP47FEB_TypeDef *dac, uint8_t REG, uint16_t DATA);
static void _WriteAddr(MCP47FEB_TypeDef *dac, uint8_t REG, uint8_t data);

/**
 * @brief 			Initialise the MCP47FEB DAC device.
 * 
 * @param dac		Struct pointer 
 * @param devAddr 	Device I2C device.
 * @param hi2c 		I2C Handle pointer
 */
void MCP47FEB_Init(MCP47FEB_TypeDef *dac, uint8_t devAddr, I2C_HandleTypeDef *hi2c) {
	dac->devAddr = devAddr;
	dac->hi2c = hi2c;
}

/**
 * @brief 						Pings the I2C Device, checking for an ACK
 * 
 * @param dac 					Struct pointer
 * @return HAL_StatusTypeDef	HAL_OK if ready (0);
 */
HAL_StatusTypeDef MCP47FEB_IsReady(MCP47FEB_TypeDef *dac) {
	return HAL_I2C_IsDeviceReady(dac->hi2c, dac->devAddr<<1, 2, 2);
}

/**
 * @brief 		Read the EEPROM Address
 * 
 * @param dac 
 * @param REG 
 * @param buffer 
 */
static void _ReadEpAddr(MCP47FEB_TypeDef *dac, uint8_t REG, uint8_t buffer[5]) {
	uint8_t readReg = 0x80 | (READ | (REG << 3));
	HAL_I2C_Master_Transmit(dac->hi2c, dac->devAddr<<1, &readReg, 1, MCP47FEB_I2C_DELAY);
	HAL_I2C_Master_Receive(dac->hi2c, dac->devAddr<<1, buffer, 2, MCP47FEB_I2C_DELAY);
}

static void _ReadAddr(MCP47FEB_TypeDef *dac, uint8_t REG, uint8_t buffer[5]) {
	uint8_t readReg = READ | (REG << 3);
	HAL_I2C_Master_Transmit(dac->hi2c, dac->devAddr<<1, &readReg, 1, MCP47FEB_I2C_DELAY);
	HAL_I2C_Master_Receive(dac->hi2c, dac->devAddr<<1, buffer, 2, MCP47FEB_I2C_DELAY);
}


static void _FastWrite(MCP47FEB_TypeDef *dac, uint8_t REG, uint16_t DATA) {
	uint8_t payload[8];
	payload[0] = (REG << 3) | WRITE;
	payload[1] = (DATA >> 8) & 0xFF;
	payload[2] = (DATA & 0xFF);
	HAL_I2C_Master_Transmit(dac->hi2c, dac->devAddr<<1, (uint8_t*)&payload, 3, MCP47FEB_I2C_DELAY);
}


static void _WriteAddr(MCP47FEB_TypeDef *dac, uint8_t REG, uint8_t data) {
	uint8_t payload[3];
	payload[0] = (REG << 3) | WRITE;
	if (REG == GAIN_REG) {
		payload[2] = 0;
		payload[1] = data;
	} else {
		payload[1] = 0;
		payload[2] = data;
	}
	HAL_I2C_Master_Transmit(dac->hi2c, dac->devAddr<<1, (uint8_t*)&payload, 3, MCP47FEB_I2C_DELAY);

}


void MCP47FEB_UnlockSALCK(MCP47FEB_TypeDef *dac) {
	//SET HVC PIN LOW
	_WriteAddr(dac, (SALCK | UNLOCK_SALCK), 0);
	// SET HVC PIN LOW
}

void MCP47FEB_LockSALCK(MCP47FEB_TypeDef *dac, uint8_t addr) {
	//SET HVC PIN HIGH
	/* Create new struct with new addr */
	MCP47FEB_TypeDef BIAS_DAC = {
		addr,
		dac->hi2c,
	};

	_WriteAddr(&BIAS_DAC, (SALCK | UNLOCK_SALCK), 0);
	//SET HVC PIN LOW
}
/**
 * @brief 			Get the Power-Down control bits for a channel
 * 					0x00 = Normal Operation
 * 					0x01 = 1kOhm resistor to ground
 * 					0x10 = 100kOhm resistor to ground
 * 					0x11 = Open Circuit
 * @param dac		DAC Struct 
 * @param channel 	Channel to read
 * @return uint8_t 	Control Bits. 
 */
uint8_t MCP47FEB_GetPowerDown(MCP47FEB_TypeDef *dac, uint8_t channel) {
	_ReadAddr(dac, PD_REG, readBuffer);
	uint8_t _powerDown[2];
	_powerDown[0] = (readBuffer[1] & 0x03); 		// 0B00000011
	_powerDown[1] = (readBuffer[1] & 0x0C) >> 2; 	// 0B00001100) >> 2;
	return (channel == 0) ? _powerDown[0] : _powerDown[1];
}

/**
 * @brief 		Set the Power-Down control bits
 * 					0x00 = Normal Operation
 * 					0x01 = 1kOhm resistor to ground
 * 					0x10 = 100kOhm resistor to ground
 * 					0x11 = Open Circuit
 * @param dac 	DAC Struct
 * @param val0 	Channel 0 Bits
 * @param val1 	Channel 1 Bits
 */
void MCP47FEB_SetPowerDown(MCP47FEB_TypeDef *dac, uint8_t val0, uint8_t val1) {
	_WriteAddr(dac, PD_REG, (val0 | val1<<2));
}


uint8_t MCP47FEB_GetPowerDownEp(MCP47FEB_TypeDef *dac, uint8_t channel) {
	_ReadEpAddr(dac, PD_REG, readBuffer);
	uint8_t _powerDownEp[2];
	_powerDownEp[0] = (readBuffer[1] & 0x03); 			// 0B00000011);
	_powerDownEp[1] = (readBuffer[1] & 0x0C) >> 2;		// 0B00001100) >> 2;
	return (channel == 0) ? _powerDownEp[0] : _powerDownEp[1];
}

/**
 * @brief 			Get the Gain Control bit for a given channel
 * 
 * @param dac 		DAC Struct
 * @param channel 	Channel
 * @return uint8_t 	Gain Bits set. 1 = 2x Gain, 0 = 1x Gain
 */
uint8_t MCP47FEB_GetGain(MCP47FEB_TypeDef *dac, uint8_t channel) {
	uint8_t buff[5] = {0};
	_ReadAddr(dac, GAIN_REG, buff);
	uint8_t _gain[2];
	_gain[0] = (buff[0] & 0x01); 	//0B00000001);
	_gain[1] = (buff[0] & 0x02) >> 1; 	// 0B00000010)>>1;
	return (channel == 0) ? _gain[0] : _gain[1];
}

/**
 * @brief 		Sets the Gain Control bits (Datasheet p.39)
 *	 			
 * @param dac 	DAC Struct
 * @param val0	DAC0 Gain control. 1 = 2x Gain, 0 = 1x Gain
 * @param val1 	DAC1 Gain contron. 1 = 2x Gain, 0 = 1x Gain
 */
void MCP47FEB_SetGain(MCP47FEB_TypeDef *dac, uint8_t val0, uint8_t val1) {
	_WriteAddr(dac, GAIN_REG, (val0 | (val1<<1)));
}

uint8_t MCP47FEB_GetGainEp(MCP47FEB_TypeDef *dac, uint8_t channel) {
	_ReadEpAddr(dac, GAIN_REG, readBuffer);
	uint8_t _gainEp[2];
	_gainEp[0] = (readBuffer[0] & 0x01);		//0B00000001);
	_gainEp[1] = (readBuffer[0] & 0x02) >> 1;	//0B00000010)>>1;
	return (channel == 0) ? _gainEp[0] : _gainEp[1];
}

/**
 * @brief 			Get the Voltage Reference Control Register for a channel
 * 					(Datasheet register 4-2)
 * 					0b11 = Vref buffer enabled
 * 					0b10 = Vref buffer disabled
 * 					0b01 = Vref buffer enabled. Voltage driven when powered down
 * 					0b00 = Vref buffer disabled. Lowest current
 * @param dac 		DAC Struct
 * @param channel 	Channel to read
 * @return uint8_t 	Voltage Reference
 */
uint8_t MCP47FEB_GetVref(MCP47FEB_TypeDef *dac, uint8_t channel) {
	_ReadAddr(dac,VREF_REG, readBuffer);
	uint8_t _intVref[2];
	_intVref[0] = (readBuffer[1] & 0x03); 		//0b00000011);
	_intVref[1] = (readBuffer[1] & 0x0C) >> 2; 	//0b00001100) >> 2;
	return (channel == 0) ? _intVref[0] : _intVref[1];
}

/**
 * @brief 		Set the Voltage Reference Control Register for each channel
 * 					(Datasheet register 4-2)
 * 					0b11 = Vref buffer enabled
 * 					0b10 = Vref buffer disabled
 * 					0b01 = Vref buffer enabled. Voltage driven when powered down
 * 					0b00 = Vref buffer disabled. Lowest current
 * 
 * @param dac 	DAC Struct
 * @param val0 	Channel0 value to write
 * @param val1 	Channel1 value to write
 */
void MCP47FEB_SetVref(MCP47FEB_TypeDef *dac, uint8_t val0, uint8_t val1) {
	_WriteAddr(dac, VREF_REG, (val0 | (val1<<2)));
}

/**
 * @brief 			Get the Voltage Reference stored in the EEPROM
 * 
 * @param dac 		DAC Struct
 * @param channel 	Channel to read
 * @return uint8_t 	Vref control bits.
 */
uint8_t MCP47FEB_GetVrefEp(MCP47FEB_TypeDef *dac, uint8_t channel) {//uint8_t channel) {
	_ReadEpAddr(dac, VREF_REG, readBuffer);
	uint8_t _intVrefEp[2];
	_intVrefEp[0] = (readBuffer[1] & 0x03); 		//0b00000011);
	_intVrefEp[1] = (readBuffer[1] & 0x0C) >> 2;	//0b00001100) >> 2;
	return (channel == 0) ? _intVrefEp[0] : _intVrefEp[1];
}

/**
 * @brief 			Gets the DAC value set to a channel (0-4096)
 * 
 * @param dac 		DAC Struct
 * @param channel 	Channel to read
 * @return uint16_t Channel's value (0-4096)
 */
uint16_t MCP47FEB_GetValue(MCP47FEB_TypeDef *dac, uint8_t channel) {
	_ReadAddr(dac, channel, readBuffer);
	return word((readBuffer[0] & 0x0F), readBuffer[1]);
}

/**
 * @brief 			Write value to the DAC Channels (0-4096)
 * 
 * @param dac 		DAC Struct
 * @param val0 		Channel0 Value to write 
 * @param val1 		Channel1 Value to write
 */
void MCP47FEB_AnalogWrite(MCP47FEB_TypeDef *dac, uint16_t val0, uint16_t val1) {
	val0 &= 0xFFF;
	val1 &= 0xFFF; //Prevent going over 4095
	_FastWrite(dac, DAC0_REG, val0);
	_FastWrite(dac, DAC1_REG, val1);
}

/**
 * @brief 		Write the current values of the DAC to it's EEPROM
 * 
 * @param dac 	DAC Struct
 */
void MCP47FEB_EEPROMWrite(MCP47FEB_TypeDef *dac) {
	_FastWrite(dac, DAC0_EP_REG, MCP47FEB_GetValue(dac, 0));
	_FastWrite(dac, DAC1_EP_REG, MCP47FEB_GetValue(dac, 1));

	uint16_t vref0 = MCP47FEB_GetVref(dac, 0);
	uint16_t vref1 = MCP47FEB_GetVref(dac, 1);

	_FastWrite(dac, VREF_EP_REG, (vref0 | vref1 << 2));//(MCP47FEB_GetVref(dac,0) | MCP47FEB_GetVref(dac,1)<<2));
	_FastWrite(dac, GAIN_EP_REG, (MCP47FEB_GetGain(dac, 0) | MCP47FEB_GetGain(dac, 1)<<1)<<8);
	_FastWrite(dac, PD_EP_REG, (MCP47FEB_GetPowerDown(dac, 0) | MCP47FEB_GetPowerDown(dac, 1)<<2));
}
