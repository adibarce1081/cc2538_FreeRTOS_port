
#include "adc_tempsens.h"

#define CONST 0.58134 //(VREF / 2047) = (1190 / 2047), VREF from Datasheet
#define OFFSET_DATASHEET_25C 827 // 1422*CONST, from Datasheet 
#define TEMP_COEFF (CONST * 4.2) // From Datasheet
#define OFFSET_0C (OFFSET_DATASHEET_25C - (25 * TEMP_COEFF))

float internal_soc_temperature(void) {
    
	uint16_t ui16Dummy;
    	float dOutputVoltage;

    	//SysCtrlClockSet(false, false, SYS_CTRL_SYSDIV_32MHZ);

    	//SysCtrlIOClockSet(SYS_CTRL_SYSDIV_32MHZ);
     
    	SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_RFC);
	
	HWREG(CCTEST_TR0) |= CCTEST_TR0_ADCTM;

    	HWREG(RFCORE_XREG_ATEST) = 0x01;
    
    	SOCADCSingleConfigure(SOCADC_12_BIT, SOCADC_REF_INTERNAL);

        SOCADCSingleStart(SOCADC_TEMP_SENS);
        
        while(!SOCADCEndOfCOnversionGet()){
        }

        ui16Dummy = SOCADCDataGet() >> SOCADC_12_BIT_RSHIFT;
        
        dOutputVoltage = ui16Dummy * CONST;       
        
	dOutputVoltage = ((dOutputVoltage - OFFSET_0C) / TEMP_COEFF);
        
	return dOutputVoltage;
}
