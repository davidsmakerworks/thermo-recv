/*
 * Remote Temperature Sensor Receiver Module
 * Copyright (c) 2019 David Rice
 * 
 * Processor: PIC16F18325
 * 
 * The following macros must be defined on the XC8 command line or in project properties:
 * _XTAL_FREQ - CPU speed in Hz (Fosc) - must be 32000000 for this driver
 * RF_CHANNEL - the channel to use for the RF module (example: 0x10U)
 * MAX_LED_INDEX - the maximum index of the connected to the driver (i.e., number of LEDs - 1)
 * LED_BRIGHTNESS - the brightness of the LEDs on the strip (range: 0x00 to 0xFF)
 * 
 * Additionally, exactly one of the following must be defined (without a value) to specify 
 * the type of LEDs connected to the driver:
 * WS2811
 * WS2812B
 * 
 * Drivers used:
 * NRF24L01P
 * 
 * Peripheral usage:
 * MSSP1 - Drives WS281x LED strip (SPI master mode)
 * MSSP2 - Communication with RF module (SPI master mode)
 * Timer2 - PWM clock source for WS281x protocol
 * PWM5 - Generates PWM signal for WS281x protocol
 * CLC1 - Combines MSSP1 and PWM5 signals to produce WS2812x protocol
 * 
 * Pin assignments:
 * RA2 - WS2812x data output from CLC
 * RA4 - Reset button
 * RC0 - RF module CE
 * RC1 - RF module CSN
 * RC2 - RF module SCK
 * RC3 - RF module MOSI
 * RC4 - RF module MISO
 * RC5 - RF module IRQ
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// PIC16F18325 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // FEXTOSC External Oscillator mode Selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with 2x PLL (32MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR/VPP pin function is MCLR; Weak pull-up enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config WDTE = OFF       // Watchdog Timer Enable bits (WDT disabled; SWDTEN is ignored)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled, SBOREN bit ignored)
#pragma config BORV = LOW       // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.45V)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (The PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a Reset)
#pragma config DEBUG = OFF      // Debugger enable bit (Background debugger disabled)

// CONFIG3
#pragma config WRT = OFF        // User NVM self-write protection bits (Write protection off)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored.)

// CONFIG4
#pragma config CP = OFF         // User NVM Program Memory Code Protection bit (User NVM code protection disabled)
#pragma config CPD = OFF        // Data NVM Memory Code Protection bit (Data NVM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "nRF24L01P.h"
#include "nRF24L01P-cfg.h"

#ifndef RF_CHANNEL
#error RF_CHANNEL must be defined
#endif

#ifndef LED_BRIGHTNESS
#error LED_BRIGHTNESS must be defined
#endif

#ifndef MAX_LED_INDEX
#error MAX_LED_INDEX must be defined
#endif

#if _XTAL_FREQ != 32000000
#error _XTAL_FREQ must be defined as 32000000 (this driver requires Fosc = 32 MHz)
#endif

/* 
 * Macros for putting color values at the appropriate index
 * Note that the WS2812B uses GRB format instead of RGB
 */
#ifdef WS2811
#define RED(x)   ((x) * 3)
#define GREEN(x) ((x) * 3) + 1
#define BLUE(x)  ((x) * 3) + 2
#else
#ifdef WS2812B
#define RED(x)   ((x) * 3) + 1
#define GREEN(x) ((x) * 3)
#define BLUE(x)  ((x) * 3) + 2
#else
#error LED controller type must be specified
#endif
#endif

#define ADDR_LEN        5
#define PAYLOAD_WIDTH   2

uint8_t color_data[(MAX_LED_INDEX + 1) * 3];

const uint8_t display_addr[ADDR_LEN] = { 'T', 'E', 'M', 'P', 0xA5 };

/* 
 * Standard port initialization
 */
void init_ports(void) {
    /* Disable all analog features */
    ANSELA = 0x00;
    ANSELC = 0x00;
    
    /* Enable weak pull-up for reset button on RA4 */
    WPUA = _WPUA_WPUA4_MASK;
    
    /* Pull all outputs low except RC1 (RF_CSN) */
    LATA = 0x00;
    LATC = _LATC_LATC1_MASK;
    
    /* Set all ports to output except RA4 (reset button), RC4 (SDI2), and RC5 (RF_IRQ) */
    TRISA = _TRISA_TRISA4_MASK;
    TRISC = _TRISC_TRISC4_MASK |
            _TRISC_TRISC5_MASK;
    
    /* Set TTL on RC4 (SDI2) and RC5 (RF_IRQ) due to 3.3V output from RF module */
    INLVLCbits.INLVLC4 = 0;
    INLVLCbits.INLVLC5 = 0;
}

/* Initialize MSSP modules that will drive the RF module and CLC */
void init_mssp(void) {
    /* MSSP2 used to drive RF module */   
    /* Set MSSP2 to SPI Master mode using baud rate generator */
    SSP2CON1bits.SSPM = 0b1010;
    
    /* 1 MHz at Fosc = 32 MHz */
    SSP2ADD = 7;
    
    /* Transmit data on active-to-idle transition */
    SSP2STATbits.CKE = 1;
    
    /* Enable MSSP2 */
    SSP2CON1bits.SSPEN = 1;
    
    /* MSSP1 used only to drive CLC */
    /* Set SPI mode with CLK = T2_match/2 */
    SSP1CON1bits.SSPM = 0b0011;
    
    /* Enable MSSP */
    SSP1CON1bits.SSPEN = 1; 
}

/* Initialize Timer2 to be PWM clock source */
void init_timers(void) {
    /* 0.625 uSec at Fosc = 32 MHz */
    PR2 = 4;
    
    /* Enable Timer2 */
    T2CONbits.TMR2ON = 1; 
}

/* Initialize PWM generator for WS281x zero-bit pulses */
void init_pwm(void) {
    PWM5DCH = 1;
    PWM5DCL = 0;
    
    PWM5CONbits.PWM5EN = 1; /* Enable PWM generator */
}

/* Set up CLC1 to output (SCK && SDO) || (nSDO && SCK && PWM) */
void init_clc(void) {

    /* CLC1 input 1 is SDO1 */
    CLC1SEL0bits.LC1D1S = 0b10011;
    
    /* CLC1 input 2 is SCK1 */
    CLC1SEL1bits.LC1D2S = 0b10010;
    
    /* CLC1 input 3 is PWM5OUT */
    CLC1SEL2bits.LC1D3S = 0b10000; 
    
    /* Gate behavior is undefined at power-on so must be set to zero */
    CLC1GLS0 = 0x00;
    CLC1GLS1 = 0x00;
    CLC1GLS2 = 0x00;
    CLC1GLS3 = 0x00; 
    
    /* SDO input to AND gate 1 */
    CLC1GLS0bits.LC1G1D1T = 1; 
    
    /* SCK input to AND gate 1 */
    CLC1GLS1bits.LC1G2D2T = 1; 
    
    /* nSDO && SCK = n(SDO || nSCK) */
    /* SDO || nSCK input to AND gate 2 */
    CLC1GLS2bits.LC1G3D1T = 1;
    CLC1GLS2bits.LC1G3D2N = 1; 
    
    /* PWM5OUT input to AND gate 2 */
    CLC1GLS3bits.LC1G4D3T = 1; 
    
    /* Clear all inversion bits */
    CLC1POL = 0x00;
    
    /* Gate 3 n(SDO || nSCK) is inverted to obtain (nSDO && SDK) */
    CLC1POLbits.LC1G3POL = 1; 
    
    /* Enable CLC1 */
    CLC1CONbits.LC1EN = 1; 
}

/* Initialize PPS to route signals to pins */
void init_pps(void) {
    bool state;
    
    /* Preserve global interrupt state and disable interrupts */
    state = INTCONbits.GIE;
    INTCONbits.GIE = 0;
    
    /* Unlock PPS */
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0;
    
    /* SCK2 on RC2 */
    RC2PPS = 0b11010;
    SSP2CLKPPS = 0b10010;
    
    /* SDI2 on RC4 */
    SSP2DATPPS = 0b10100;
    
    /* SDO2 on RC3 */
    RC3PPS = 0b11011;
    
    /* CLC1OUT on RA2 */
    RA2PPS = 0b00100;
    
    /* Lock PPS */
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 1;
    
    /* Restore global interrupt state */
    INTCONbits.GIE = state;
}

void init_rf(void) {
    /* Allow for maximum possible RF module startup time */
    __delay_ms(100);
    
    /* Set 1-byte payload width */
    nrf24_write_register(NRF24_RX_PW_P0, PAYLOAD_WIDTH);
    
    /* Set RF power to 0 dBm and data rate to 1 Mbit/Sec */
    nrf24_write_register(NRF24_RF_SETUP, NRF24_RF_PWR_0DBM);
    
    /* Set 5-byte address width */
    nrf24_write_register(NRF24_SETUP_AW, NRF24_AW_5);
    
    /* Set initial RF channel */
    nrf24_write_register(NRF24_RF_CH, RF_CHANNEL);
    
    /* Enable receive on pipe 0 only */
    nrf24_write_register(NRF24_EN_RXADDR, NRF24_ERX_P0);
    
    /* Set receive address  */
    nrf24_set_rx_address(NRF24_RX_ADDR_P0, display_addr, ADDR_LEN);
    
    /* Mask RX_DR interrupt on RF module, enable CRC, power up RF module in transmit-standby mode */
    nrf24_write_register(NRF24_CONFIG, NRF24_MASK_TX_DS | NRF24_MASK_MAX_RT | NRF24_EN_CRC | NRF24_PWR_UP | NRF24_PRIM_RX);
    
    /* Clear any pending RF module interrupts */
    nrf24_write_register(NRF24_STATUS, NRF24_TX_DS | NRF24_MAX_RT | NRF24_RX_DR);
}

/* Transfer one byte of data on MSSP2 for the RF module */
uint8_t transfer_spi(uint8_t data) {
    SSP2BUF = data;
    
    while (!SSP2STATbits.BF);
    
    data = SSP2BUF;
    
    return data;
}

/* 
 * Send all LED data on MSSP1
 *
 * This function blocks until all data is transmitted.
 */
void send_led_data(void) {
    uint16_t current_index;
    
    for (current_index = 0; current_index < (MAX_LED_INDEX + 1) * 3; current_index++) {
        SSP1BUF = color_data[current_index];
        
        while (!SSP1STATbits.BF);
    }
}

void all_off(void) {
    uint16_t current_led;
    
    for (current_led = 0; current_led < MAX_LED_INDEX + 1; current_led++) {
        color_data[RED(current_led)] = 0x00;
        color_data[GREEN(current_led)] = 0x00;
        color_data[BLUE(current_led)] = 0x00;
    }
}

void exact_temp_match(void) {
    all_off();
    
    color_data[GREEN(((MAX_LED_INDEX + 1) / 2) - 1)] = LED_BRIGHTNESS;
    color_data[GREEN((MAX_LED_INDEX + 1) / 2)] = LED_BRIGHTNESS;
}

void temp_high(uint16_t diff) {
    uint16_t start_index;
    uint16_t end_index;
    uint16_t current_led;
    
    all_off();
    
    if (diff > (MAX_LED_INDEX + 1) / 2) {
        diff = (MAX_LED_INDEX + 1) / 2;
    }
    
    start_index = ((MAX_LED_INDEX + 1) / 2) - diff;
    end_index = start_index + ((diff * 2) - 1);
    
    for (current_led = start_index; current_led <= end_index; current_led++) {
        color_data[RED(current_led)] = LED_BRIGHTNESS;
    }
}
    
    
void temp_low(uint16_t diff) {
    uint16_t start_index;
    uint16_t end_index;
    uint16_t current_led;
    
    all_off();
    
    if (diff > (MAX_LED_INDEX + 1) / 2) {
        diff = (MAX_LED_INDEX + 1) / 2;
    }
    
    start_index = ((MAX_LED_INDEX + 1) / 2) - diff;
    end_index = start_index + ((diff * 2) - 1);
    
    for (current_led = start_index; current_led <= end_index; current_led++) {
        color_data[BLUE(current_led)] = LED_BRIGHTNESS;
    }
}
 

void main(void) {
    bool update_needed = false;
    
    int16_t temperature = 0;
    int16_t last_temperature = 0;
    int16_t target_temperature = 0x7FFF;
    
    /* Initialize peripherals */
    init_ports();
    init_pps();
    init_timers();
    init_pwm();
    init_clc();
    init_mssp();
    init_rf();
    
    NRF24_CE_ACTIVE();
    
    all_off();
    update_needed = true;
    
    while (1) {
        if (update_needed) {
            send_led_data();
            update_needed = false;
        }
        
        if (!PORTAbits.RA4) {
            target_temperature = temperature;
            last_temperature = 0x7FFF;
            
            exact_temp_match();
            update_needed = true;
        }
        
        if (!NRF24_IRQ) {
            nrf24_read_payload(&temperature, PAYLOAD_WIDTH);
            nrf24_flush_rx();
            
            temperature >>= 1;
            
            if (temperature != last_temperature) {
                if (temperature > target_temperature) {
                    temp_high(temperature - target_temperature);
                    update_needed = true;
                } else if (temperature < target_temperature) {
                    temp_low(target_temperature - temperature);
                    update_needed = true;
                } else {
                    exact_temp_match();
                    update_needed = true;
                }
            }
            
            last_temperature = temperature;
            
            nrf24_write_register(NRF24_STATUS, NRF24_RX_DR);            
        }
    }
}
