#include "SPI.h"
#include "SysTimer.h"
#include "accelerometer.h"

void accWrite(uint8_t addr, uint8_t val) {
		uint16_t bitstring = 0;
		bitstring |= (addr << 8) | val;	
		SPI_Transfer_Data(bitstring);
}

uint8_t accRead(uint8_t addr) {
		// access SPI_Transfer_Data
		uint16_t bitstring = 0;
		bitstring |= (addr << 8) | (1UL << 15);	
		return SPI_Transfer_Data(bitstring);
}

void initAcc(void) {
		accWrite(0x31,((1UL << 3) | 3UL));
		accWrite(0x2d, 1UL << 3);
}

int16_t twosCompToDec(uint16_t two_compliment_val)
{
    uint16_t sign_mask = 0x8000;

    if ( (two_compliment_val & sign_mask) == 0 )
        return two_compliment_val;
    else
        return -(~two_compliment_val + 1);
}

void readACCValues(double* x, double* y, double* z) {
    // Find scaler from data sheet
		uint16_t receivedData = accRead(0x32);
	
    // Read values into x, y, z using accRead
	
    int16_t rawX = (accRead(0x33)<<8)|accRead(0x32);  
    int16_t rawY = (accRead(0x35)<<8)|accRead(0x34);
    int16_t rawZ = (accRead(0x37)<<8)|accRead(0x36);

    // Calculate the scaled values using the appropriate scaler
    *x = twosCompToDec(rawX)/31.2;
    *y = twosCompToDec(rawY)/31.2;
    *z = twosCompToDec(rawZ)/31.2;
}
