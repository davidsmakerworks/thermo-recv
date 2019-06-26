/* 
 * Application-specific configuration values for nRF24L01+
 *
 * Requires definitions for:
 * NRF24_CSN_ACTIVE() - Macro to set CSN pin low
 * NRF24_CSN_IDLE() - Macro to set CSN pin high
 * NRF24_CE_ACTIVE() - Macro to set CE pin high
 * NRF24_CE_IDLE() - Macro to set CE pin low
 * NRF24_IRQ - Interrupt pin (active low) on nRF24L01+
 * NRF24_XFER_SPI(x) - Transfer one byte to/from SPI bus without changing CSN
 * 
 * Modify includes as necessary for processor architecture
 * 
 */

#ifndef NRF24L01P_CFG_H
#define	NRF24L01P_CFG_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <xc.h>

#define NRF24_CSN_ACTIVE()  LATCbits.LATC1 = 0
#define NRF24_CSN_IDLE()    LATCbits.LATC1 = 1    
#define NRF24_CE_ACTIVE()   LATCbits.LATC0 = 1
#define NRF24_CE_IDLE()     LATCbits.LATC0 = 1
#define NRF24_IRQ           PORTCbits.RC5

#define NRF24_XFER_SPI(x)   transfer_spi(x)

/* Function prototype for definition of NRF24_XFER_SPI(x) must be specified here */    
uint8_t transfer_spi(uint8_t data);
    
#ifdef	__cplusplus
}
#endif

#endif	/* NRF24L01P_CFG_H */
