#ifndef MCP47FEB_H_
#define MCP47FEB_H_

#include "stdint.h"
#include "i2c.h"

/** I2C Timeout delay **/
#define MCP47FEB_I2C_DELAY	1000


/**
 * @brief MCP47FEB Struct
 * 
 */
typedef struct {
	uint8_t 			devAddr;
	I2C_HandleTypeDef 	*hi2c;
} MCP47FEB_TypeDef;




void 	MCP47FEB_Init(MCP47FEB_TypeDef *dac, uint8_t devAddr, I2C_HandleTypeDef *hi2c);
void  	MCP47FEB_UnlockSALCK(MCP47FEB_TypeDef *dac);
void  	MCP47FEB_LockSALCK(MCP47FEB_TypeDef *dac, uint8_t addr);
uint8_t MCP47FEB_GetPowerDown(MCP47FEB_TypeDef *dac, int channel);
void 	MCP47FEB_SetPowerDown(MCP47FEB_TypeDef *dac, int val0, int val1);
uint8_t MCP47FEB_GetPowerDownEp(MCP47FEB_TypeDef *dac, int channel);
uint8_t MCP47FEB_GetGain(MCP47FEB_TypeDef *dac, int channel);
void 	MCP47FEB_SetGain(MCP47FEB_TypeDef *dac, int val0, int val1);
uint8_t MCP47FEB_GetGainEp(MCP47FEB_TypeDef *dac, int channel);
uint8_t MCP47FEB_GetVref(MCP47FEB_TypeDef *dac, uint8_t channel);
void 	MCP47FEB_SetVref(MCP47FEB_TypeDef *dac, uint8_t val0, uint8_t val1);
uint8_t MCP47FEB_GetVrefEp(MCP47FEB_TypeDef *dac, uint8_t channel);
uint16_t MCP47FEB_GetValue(MCP47FEB_TypeDef *dac, uint8_t channel);
void 	MCP47FEB_AnalogWrite(MCP47FEB_TypeDef *dac, uint16_t val0, uint16_t val1);
void 	MCP47FEB_EEPROMWrite(MCP47FEB_TypeDef *dac);


#endif
