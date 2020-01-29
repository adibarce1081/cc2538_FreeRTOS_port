
#include "bsp.h"
#include "ieee_addr.h"

#define IEEE_ADDR_OUI_TI   { 0x00, 0x12, 0x4B }
#define IEEE_ADDR_LOC	0x00280028

void getIEEEAddrFrmFlash(uint8_t *ieee_address){

	uint8_t i;
	uint8_t oui_ti[3] = IEEE_ADDR_OUI_TI;
	uint8_t len = 8;

    if(((uint8_t *)IEEE_ADDR_LOC)[3] == oui_ti[0]
       && ((uint8_t *)IEEE_ADDR_LOC)[2] == oui_ti[1]
       && ((uint8_t *)IEEE_ADDR_LOC)[1] == oui_ti[2]) {
      for(i = 0; i < len / 2; i++) {
        ieee_address[i] = ((uint8_t *)IEEE_ADDR_LOC)[len / 2 - 1 - i];
      }
      for(i = 0; i < len / 2; i++) {
    	ieee_address[i + len / 2] = ((uint8_t *)IEEE_ADDR_LOC)[len - 1 - i];
      }
    } else {
      for(i = 0; i < len; i++) {
        ieee_address[i] = ((uint8_t *)IEEE_ADDR_LOC)[len - 1 - i];
      }
    }
}
