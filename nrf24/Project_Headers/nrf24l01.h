/*
 * nrf24l01.h
 *
 *  Created on: Feb 21, 2017
 *      Author: Danchi
 */

#ifndef NRF24L01_H_
#define NRF24L01_H_

//Custom design for CE and CSN
#define CE_EN (1<<3)
#define CSN_EN (1<<4)
#define GREEN_LED (1<<19)
#define RED_LED (1<<18)

#define W
#define R
#define RED_LED_ON GPIOB_PCOR|= (RED_LED)
#define RED_lED_OFF GPIOB_PSOR|= (RED_LED)

#define GREEN_LED_ON GPIOB_PCOR|= (GREEN_LED)
#define GREEN_lED_OFF GPIOB_PSOR|= (GREEN_LED)

//common functions declarations
void spi_init();
void CSN_ENABLE();
void CSN_DISABLE();
void CE_ENABLE();
void CE_DISABLE();

uint8_t spi_send_byte(uint8_t data);
uint8_t *write_to_nrf(uint8_t ReadWrite, uint8_t reg,uint8_t *val, uint8_t intVal);
uint8_t nrf24_read_register(uint8_t regnumber);
uint8_t nrf24_write_register(uint8_t regnumber);
void nrf24_init();
void transmit_payload(uint8_t *w_buff);
void receive_payload(void);
void rgb_init();
void reset(void);
uint8_t get_status();




//SPI function to read data
uint8_t SPI_ReadWriteData(uint8_t *pRead,uint8_t *pWrite,uint32_t uiLength);

////////////////////////////////////////////////////////////////////////////////////
// SPI commands
//
// The following are defines for all of the commands and data masks on the SPI 
//   interface.
////////////////////////////////////////////////////////////////////////////////////
//SPI command defines
#define nrf24l01_R_REGISTER		0x00
#define nrf24l01_W_REGISTER		0x20
#define nrf24l01_R_RX_PAYLOAD	0x61
#define nrf24l01_W_TX_PAYLOAD	0xA0
#define nrf24l01_FLUSH_TX		0xE1
#define nrf24l01_FLUSH_RX		0xE2
#define nrf24l01_REUSE_TX_PL	0xE3
#define nrf24l01_NOP			0xFF
#define nrf24l01_R_RX_PL_WID	0X60
#define nrf24l01_W_ACK_PAYLOAD	0xA8
#define nrf24l01_W_ACK_PAYLOAD_NOACK	0xB0


//SPI command data mask defines
#define nrf24l01_R_REGISTER_DATA	0x1F
#define nrf24l01_W_REGISTER_DATA	0x1F

////////////////////////////////////////////////////////////////////////////////////
// Register definitions
//
// Below are the defines for each register's address in the 24L01.
////////////////////////////////////////////////////////////////////////////////////
#define nrf24l01_CONFIG			0x00
#define nrf24l01_EN_AA			0x01
#define nrf24l01_EN_RXADDR		0x02
#define nrf24l01_SETUP_AW		0x03
#define nrf24l01_SETUP_RETR		0x04
#define nrf24l01_RF_CH			0x05
#define nrf24l01_RF_SETUP		0x06
#define nrf24l01_STATUS			0x07
#define nrf24l01_OBSERVE_TX		0x08
#define nrf24l01_RPD			0x09
#define nrf24l01_RX_ADDR_P0		0x0A
#define nrf24l01_RX_ADDR_P1		0x0B
#define nrf24l01_RX_ADDR_P2		0x0C
#define nrf24l01_RX_ADDR_P3		0x0D
#define nrf24l01_RX_ADDR_P4		0x0E
#define nrf24l01_RX_ADDR_P5		0x0F
#define nrf24l01_TX_ADDR		0x10
#define nrf24l01_RX_PW_P0		0x11
#define nrf24l01_RX_PW_P1		0x12
#define nrf24l01_RX_PW_P2		0x13
#define nrf24l01_RX_PW_P3		0x14
#define nrf24l01_RX_PW_P4		0x15
#define nrf24l01_RX_PW_P5		0x16
#define nrf24l01_FIFO_STATUS	0x17
#define nrf24l01_DYNPD			0x1C
#define nrf24l01_FEATURE		0x1D


////////////////////////////////////////////////////////////////////////////////////
// Default register values
//
// Below are the defines for each register's default value in the 24L01. Multi-byte
//   registers use notation B<X>, where "B" represents "byte" and <X> is the byte
//   number.
////////////////////////////////////////////////////////////////////////////////////
#define nrf24l01_CONFIG_DEFAULT_VAL			0x08
#define nrf24l01_EN_AA_DEFAULT_VAL			0x3F
#define nrf24l01_EN_RXADDR_DEFAULT_VAL		0x03
#define nrf24l01_SETUP_AW_DEFAULT_VAL		0x03
#define nrf24l01_SETUP_RETR_DEFAULT_VAL		0x03
#define nrf24l01_RF_CH_DEFAULT_VAL			0x02
#define nrf24l01_RF_SETUP_DEFAULT_VAL		0x0F
#define nrf24l01_STATUS_DEFAULT_VAL			0x0E
#define nrf24l01_OBSERVE_TX_DEFAULT_VAL		0x00
#define nrf24l01_RPD_DEFAULT_VAL				0x00
#define nrf24l01_RX_ADDR_P0_B0_DEFAULT_VAL	0xE7
#define nrf24l01_RX_ADDR_P0_B1_DEFAULT_VAL	0xE7
#define nrf24l01_RX_ADDR_P0_B2_DEFAULT_VAL	0xE7
#define nrf24l01_RX_ADDR_P0_B3_DEFAULT_VAL	0xE7
#define nrf24l01_RX_ADDR_P0_B4_DEFAULT_VAL	0xE7
#define nrf24l01_RX_ADDR_P1_B0_DEFAULT_VAL	0xC2
#define nrf24l01_RX_ADDR_P1_B1_DEFAULT_VAL	0xC2
#define nrf24l01_RX_ADDR_P1_B2_DEFAULT_VAL	0xC2
#define nrf24l01_RX_ADDR_P1_B3_DEFAULT_VAL	0xC2
#define nrf24l01_RX_ADDR_P1_B4_DEFAULT_VAL	0xC2
#define nrf24l01_RX_ADDR_P2_DEFAULT_VAL		0xC3
#define nrf24l01_RX_ADDR_P3_DEFAULT_VAL		0xC4
#define nrf24l01_RX_ADDR_P4_DEFAULT_VAL		0xC5
#define nrf24l01_RX_ADDR_P5_DEFAULT_VAL		0xC6
#define nrf24l01_TX_ADDR_B0_DEFAULT_VAL		0xE7
#define nrf24l01_TX_ADDR_B1_DEFAULT_VAL		0xE7
#define nrf24l01_TX_ADDR_B2_DEFAULT_VAL		0xE7
#define nrf24l01_TX_ADDR_B3_DEFAULT_VAL		0xE7
#define nrf24l01_TX_ADDR_B4_DEFAULT_VAL		0xE7
#define nrf24l01_RX_PW_P0_DEFAULT_VAL		0x00
#define nrf24l01_RX_PW_P1_DEFAULT_VAL		0x00
#define nrf24l01_RX_PW_P2_DEFAULT_VAL		0x00
#define nrf24l01_RX_PW_P3_DEFAULT_VAL		0x00
#define nrf24l01_RX_PW_P4_DEFAULT_VAL		0x00
#define nrf24l01_RX_PW_P5_DEFAULT_VAL		0x00
#define nrf24l01_FIFO_STATUS_DEFAULT_VAL	0x11

////////////////////////////////////////////////////////////////////////////////////
// Register bitwise definitions
//
// Below are the defines for each register's bitwise fields in the 24L01.
////////////////////////////////////////////////////////////////////////////////////
//CONFIG register bitwise definitions
#define nrf24l01_CONFIG_RESERVED	0x80
#define	nrf24l01_CONFIG_MASK_RX_DR	0x40
#define	nrf24l01_CONFIG_MASK_TX_DS	0x20
#define	nrf24l01_CONFIG_MASK_MAX_RT	0x10
#define	nrf24l01_CONFIG_EN_CRC		0x08
#define	nrf24l01_CONFIG_CRCO		0x04
#define	nrf24l01_CONFIG_PWR_UP		0x02
#define	nrf24l01_CONFIG_PRIM_RX		0x01

//EN_AA register bitwise definitions
#define nrf24l01_EN_AA_RESERVED		0xC0
#define nrf24l01_EN_AA_ENAA_ALL		0x3F
#define nrf24l01_EN_AA_ENAA_P5		0x20
#define nrf24l01_EN_AA_ENAA_P4		0x10
#define nrf24l01_EN_AA_ENAA_P3		0x08
#define nrf24l01_EN_AA_ENAA_P2		0x04
#define nrf24l01_EN_AA_ENAA_P1		0x02
#define nrf24l01_EN_AA_ENAA_P0		0x01
#define nrf24l01_EN_AA_ENAA_NONE	0x00

//EN_RXADDR register bitwise definitions
#define nrf24l01_EN_RXADDR_RESERVED	0xC0
#define nrf24l01_EN_RXADDR_ERX_ALL	0x3F
#define nrf24l01_EN_RXADDR_ERX_P5	0x20
#define nrf24l01_EN_RXADDR_ERX_P4	0x10
#define nrf24l01_EN_RXADDR_ERX_P3	0x08
#define nrf24l01_EN_RXADDR_ERX_P2	0x04
#define nrf24l01_EN_RXADDR_ERX_P1	0x02
#define nrf24l01_EN_RXADDR_ERX_P0	0x01
#define nrf24l01_EN_RXADDR_ERX_NONE	0x00

//SETUP_AW register bitwise definitions
#define nrf24l01_SETUP_AW_RESERVED	0xFC
#define nrf24l01_SETUP_AW			0x03
#define nrf24l01_SETUP_AW_5BYTES	0x03
#define nrf24l01_SETUP_AW_4BYTES	0x02
#define nrf24l01_SETUP_AW_3BYTES	0x01
#define nrf24l01_SETUP_AW_ILLEGAL	0x00

//SETUP_RETR register bitwise definitions
#define nrf24l01_SETUP_RETR_ARD			0xF0
#define nrf24l01_SETUP_RETR_ARD_4000	0xF0
#define nrf24l01_SETUP_RETR_ARD_3750	0xE0
#define nrf24l01_SETUP_RETR_ARD_3500	0xD0
#define nrf24l01_SETUP_RETR_ARD_3250	0xC0
#define nrf24l01_SETUP_RETR_ARD_3000	0xB0
#define nrf24l01_SETUP_RETR_ARD_2750	0xA0
#define nrf24l01_SETUP_RETR_ARD_2500	0x90
#define nrf24l01_SETUP_RETR_ARD_2250	0x80
#define nrf24l01_SETUP_RETR_ARD_2000	0x70
#define nrf24l01_SETUP_RETR_ARD_1750	0x60
#define nrf24l01_SETUP_RETR_ARD_1500	0x50
#define nrf24l01_SETUP_RETR_ARD_1250	0x40
#define nrf24l01_SETUP_RETR_ARD_1000	0x30
#define nrf24l01_SETUP_RETR_ARD_750		0x20
#define nrf24l01_SETUP_RETR_ARD_500		0x10
#define nrf24l01_SETUP_RETR_ARD_250		0x00
#define nrf24l01_SETUP_RETR_ARC			0x0F
#define nrf24l01_SETUP_RETR_ARC_15		0x0F
#define nrf24l01_SETUP_RETR_ARC_14		0x0E
#define nrf24l01_SETUP_RETR_ARC_13		0x0D
#define nrf24l01_SETUP_RETR_ARC_12		0x0C
#define nrf24l01_SETUP_RETR_ARC_11		0x0B
#define nrf24l01_SETUP_RETR_ARC_10		0x0A
#define nrf24l01_SETUP_RETR_ARC_9		0x09
#define nrf24l01_SETUP_RETR_ARC_8		0x08
#define nrf24l01_SETUP_RETR_ARC_7		0x07
#define nrf24l01_SETUP_RETR_ARC_6		0x06
#define nrf24l01_SETUP_RETR_ARC_5		0x05
#define nrf24l01_SETUP_RETR_ARC_4		0x04
#define nrf24l01_SETUP_RETR_ARC_3		0x03
#define nrf24l01_SETUP_RETR_ARC_2		0x02
#define nrf24l01_SETUP_RETR_ARC_1		0x01
#define nrf24l01_SETUP_RETR_ARC_0		0x00

//RF_CH register bitwise definitions
#define nrf24l01_RF_CH_RESERVED	0x80

//RF_SETUP register bitwise definitions
#define nrf24l01_RF_SETUP_RESERVED	0xE0
#define nrf24l01_RF_SETUP_PLL_LOCK	0x10
#define nrf24l01_RF_SETUP_RF_DR		0x08
#define nrf24l01_RF_SETUP_RF_PWR	0x06
#define nrf24l01_RF_SETUP_RF_PWR_0	0x06
#define nrf24l01_RF_SETUP_RF_PWR_6 	0x04
#define nrf24l01_RF_SETUP_RF_PWR_12	0x02
#define nrf24l01_RF_SETUP_RF_PWR_18	0x00
#define nrf24l01_RF_SETUP_LNA_HCURR	0x01

//STATUS register bitwise definitions
#define nrf24l01_STATUS_RESERVED					0x80
#define nrf24l01_STATUS_RX_DR						0x40
#define nrf24l01_STATUS_TX_DS						0x20
#define nrf24l01_STATUS_MAX_RT						0x10
#define nrf24l01_STATUS_RX_P_NO						0x0E
#define nrf24l01_STATUS_RX_P_NO_RX_FIFO_NOT_EMPTY	0x0E
#define nrf24l01_STATUS_RX_P_NO_UNUSED				0x0C
#define nrf24l01_STATUS_RX_P_NO_5					0x0A
#define nrf24l01_STATUS_RX_P_NO_4					0x08
#define nrf24l01_STATUS_RX_P_NO_3					0x06
#define nrf24l01_STATUS_RX_P_NO_2					0x04
#define nrf24l01_STATUS_RX_P_NO_1					0x02
#define nrf24l01_STATUS_RX_P_NO_0					0x00
#define nrf24l01_STATUS_TX_FULL						0x01

//OBSERVE_TX register bitwise definitions
#define nrf24l01_OBSERVE_TX_PLOS_CNT	0xF0
#define nrf24l01_OBSERVE_TX_ARC_CNT		0x0F

//RPD register bitwise definitions
#define nrf24l01_RPD_RESERVED	0x00
#define nrf24l01_RPD_Bit			0x01

//RX_PW_P0 register bitwise definitions
#define nrf24l01_RX_PW_P0_RESERVED	0xC0

//RX_PW_P0 register bitwise definitions
#define nrf24l01_RX_PW_P0_RESERVED	0xC0

//RX_PW_P1 register bitwise definitions
#define nrf24l01_RX_PW_P1_RESERVED	0xC0

//RX_PW_P2 register bitwise definitions
#define nrf24l01_RX_PW_P2_RESERVED	0xC0

//RX_PW_P3 register bitwise definitions
#define nrf24l01_RX_PW_P3_RESERVED	0xC0

//RX_PW_P4 register bitwise definitions
#define nrf24l01_RX_PW_P4_RESERVED	0xC0

//RX_PW_P5 register bitwise definitions
#define nrf24l01_RX_PW_P5_RESERVED	0xC0

//FIFO_STATUS register bitwise definitions
#define nrf24l01_FIFO_STATUS_RESERVED	0x8C
#define nrf24l01_FIFO_STATUS_TX_REUSE	0x40
#define nrf24l01_FIFO_STATUS_TX_FULL	0x20
#define nrf24l01_FIFO_STATUS_TX_EMPTY	0x10
#define nrf24l01_FIFO_STATUS_RX_FULL	0x02
#define nrf24l01_FIFO_STATUS_RX_EMPTY	0x01

//DYNPD register bitwise definitions
#define nrf24l01_DYNPD_Reserved	0xC0
#define nrf24l01_DPL_P5		0x20	 
#define nrf24l01_DPL_P4		0x10
#define nrf24l01_DPL_P3		0x08
#define nrf24l01_DPL_P2		0x04
#define nrf24l01_DPL_P1		0x02
#define nrf24l01_DPL_P0		0x01

//FEATURE register bitwise definitions
#define nrf24l01_FEATURE_Reserved	0xF8
#define nrf24l01_EN_DPL				0x04
#define nrf24l01_EN_ACK_PAY			0x02
#define nrf24l01_EN_DYN_ACK			0x01



#endif /* NRF24L01_H_ */
