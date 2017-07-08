/*
 * main implementation: use this 'C' sample to create your own application
 *
 */
/*This is a Library for the NRF24l01+ connected to a frdm kl25z
 * 
 * 
 * */


/*TRANSMITTER
 * */

#include "derivative.h" /* include peripheral declarations */
#include "mcg.h"
#include "SystickDemo.h"
#include "nrf24l01.h"

int main(void)
{
	pll_init(8000000,LOW_POWER,CRYSTAL,4,24,MCGOUT);
	Init_Systick();
	spi_init();
	
	reset();
	
	nrf24_init();
	rgb_init();
	
	
	
	//write payload
	
	uint8_t w_buffer[5];
	int i;
	for (i=0;i<5;i++){
		w_buffer[i]=0x93;
			
	}
	/*uint8_t val[5]
	val[0]|=0x02;
	write_to_nrf(4,nrf24l01_FEATURE,val,1);*/
	//uint8_t *status;
	
	//status=write_to_nrf(5,nrf24l01_DYNPD,status,1);
	
	/*transmit_payload(w_buffer);
	Delay_mS(1500);*/
	
	while(1){
		transmit_payload(w_buffer);
		Delay_mS(3000);
		uint8_t *status;
		status=write_to_nrf(5,nrf24l01_STATUS,status,1);
		int stat;
		stat=*status;
		//transmit_payload(w_buffer);
		if(stat& nrf24l01_STATUS_MAX_RT){
			reset();
			transmit_payload(w_buffer);
			//light red LED
			Delay_mS(1500);
			GPIOB_PCOR|=RED_LED;
			Delay_mS(500);
			GPIOB_PSOR|=RED_LED;
			Delay_mS(500);
			
		}
		else {
			GPIOB_PCOR|= GREEN_LED;
			Delay_mS(500);
			GPIOB_PSOR|= GREEN_LED;
			Delay_mS(500);
			reset();
			
		}
	
	}
	
	return 0;
}

/*This function initializes the SPI communication interface in kl25z as a master
 * it initializes the SPI pins PSC,SCK,MOSI,MISO
 * For the chip select pin i will use a custom 
 * one to be able to control the length of an 
 * SPI session which starts with a high to low 
 * transition of the CSN pin. 
 * */
void spi_init(){
	
	SIM_SCGC4 |= SIM_SCGC4_SPI0_MASK;

			  // enable PORT
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK|SIM_SCGC5_PORTC_MASK|SIM_SCGC5_PORTD_MASK;

				// disable SPI
	SPI0_C1 &= ~SPI_C1_SPE_MASK;

	// configure I/O to SPI function
	PORTD_PCR3 = PORT_PCR_MUX(1);  //CE
	PORTD_PCR4 = PORT_PCR_MUX(1);  //CSN
			PORTA_PCR16 &= ~PORT_PCR_MUX_MASK;
			PORTA_PCR16 |= PORT_PCR_MUX(5)|PORT_PCR_DSE_MASK;			  //Use PTA16 as SPI0_MISO
	PORTA_PCR17 &= ~PORT_PCR_MUX_MASK;
			PORTA_PCR17 |= PORT_PCR_MUX(5)|PORT_PCR_DSE_MASK;			  //Use PTA17 as SPI0_MOSI
	PORTC_PCR5 &= ~PORT_PCR_MUX_MASK;
			PORTC_PCR5 |= PORT_PCR_MUX(2)|PORT_PCR_DSE_MASK;			  //Use PTC5 as SPI0_sck
	
	  SPI0_C1 &= ~ SPI_C1_CPHA_MASK;
 //   SPI0_C1 = SPI_C1_MSTR_MASK | SPI_C1_CPOL_MASK | SPI_C1_CPHA_MASK | SPI_C1_SSOE_MASK;          /* Set configuration register */
	 SPI0_C1 = SPI_C1_MSTR_MASK ; //CPOL=0,CPHA=0 use polling
	//SPI0_C1 = SPI_C1_MSTR_MASK | SPI_C1_CPOL_MASK | SPI_C1_SSOE_MASK;//CPOL=1,CPHA=0
	//SPI0_C1 = SPI_C1_MSTR_MASK | SPI_C1_CPHA_MASK | SPI_C1_SSOE_MASK;//CPLO=0,CPHA=1
	//SPI0_C2 = SPI_C2_MODFEN_MASK;        /* Set configuration register */
	
	 SPI0_BR = (SPI_BR_SPPR(0x02) | SPI_BR_SPR(0x02)); /* Set baud rate register for baud rate of 1Mbps*/
	
	 SPI0_C1 |= SPI_C1_SPE_MASK;          /* Enable SPI module */

	//custom configuration of CSN and CE
	GPIOD_PDDR= CSN_EN;
	GPIOD_PDDR|=CE_EN;
	GPIOD_PSOR |= CSN_EN;//keep CSN initially high
	GPIOD_PCOR |= CE_EN;//keep CE initially low
}

/*Functions to Set and Clear CSN and CE
 * */
void CSN_ENABLE(){
	GPIOD_PSOR |= CSN_EN; 

}

void CSN_DISABLE(){
	GPIOD_PCOR |= CSN_EN; 
	
}

void CE_ENABLE(){
	GPIOD_PSOR |= CE_EN;
	
}
void CE_DISABLE(){
	GPIOD_PCOR |= CE_EN; 
	
}

/*SPI function to read the contents of a register
 * */
uint8_t SPI_ReadWriteData(uint8_t *pRead,uint8_t *pWrite,uint32_t uiLength)
{
		uint16_t i;


		for( i=0;i<uiLength;i++)
		{
			while(!(SPI1_S & SPI_S_SPTEF_MASK ) );
			SPI1_D = pWrite[i];
			while(!(SPI1_S & SPI_S_SPRF_MASK ) );  //read data 
			pRead[i] = SPI1_D;
		}

    return 1;
}


/*	Function to read the contents of a register
 *takes in the register number and returns
 *the contents of the register
 * 
 * 	Communication to the nrf24 chip starts with
 * a high to low transition of the CSN pin which 
 * should last for the entire length of the transmission.
 * 	Once the register address is send to the chip one sends dummy
 * values according to how much data you want to obtain from the chip
 * 
 * 	There is a 10uS delay before the chip is able to respond to data sent
 * from the master
 * 
 * */
/*
uint8_t nrf24_read_register(char regnumber){
	uint8_t data,dummy,reg_content,status;
	dummy=0xFF;
	data|=nrf24l01_R_REGISTER |regnumber;
	CSN_DISABLE();//start of SPI transmission
		Delay_mS(0.10);
			while(!(SPI1_S & SPI_S_SPTEF_MASK ) );
			SPI1_D = data;
			while(!(SPI1_S & SPI_S_SPRF_MASK ) );  //read data 
			status= SPI1_D;
		Delay_mS(0.10);	
			while(!(SPI1_S & SPI_S_SPTEF_MASK ) );
			SPI1_D = dummy;
			while(!(SPI1_S & SPI_S_SPRF_MASK ) );  //read data 
			reg_content= SPI1_D;
		Delay_mS(0.10);
			
	CSN_ENABLE();//End of SPI transmission
	
	return reg_content;
}*/

/*Function to send data on SPI
 * */
uint8_t spi_send_byte(uint8_t data){
	uint8_t reg_content;
	while(!(SPI0_S & SPI_S_SPTEF_MASK ) );
	SPI0_D = data;
	while(!(SPI0_S & SPI_S_SPRF_MASK ) );  //read data 
	reg_content= SPI0_D;
	
	return reg_content;
}
/*Function to do a general read or write operation on the nrf24
 * 
 * */
uint8_t *write_to_nrf(uint8_t ReadWrite, uint8_t reg,uint8_t *val, uint8_t intVal){
	// The ReadWrite is to set whether one wants to read or write register
	// reg is the specific register 
	// *val is an array with configuration information
	// intVal is the number of configuration information in val
	
	if (ReadWrite==4){
		reg |=nrf24l01_W_REGISTER |reg;
		
	}
	// creating an array to store returns of the nrf24
	static uint8_t nrf_return[32];
	Delay_mS(0.01);
	
	//taking CSN low
	CSN_DISABLE();//start of SPI transmission
		Delay_mS(0.01);
		spi_send_byte(reg);
		
		int i;
		//if a read operation is being done save it to an array
		//if a write operation, only write the instructions
		for (i=0;i<intVal;i++){
			if(ReadWrite==5 && reg !=nrf24l01_W_TX_PAYLOAD){
				nrf_return[i]=spi_send_byte(nrf24l01_NOP);
				Delay_mS(0.01);
			}
			else{
				spi_send_byte(val[i]);
				Delay_mS(0.01);
				
			}
		}
		CSN_ENABLE();// end of SPI transmission
		return nrf_return;
	
}
/*Function to initialize the model
 * The following registers are configured
 * CONFIG, EN_AA,EN_RXADDR,SETUP_AW, SETUP_RETR, RF_CHANNEL,
 * RF_SETUP,STATUS,RXADDR
 * 
 * The NRF@ will now be setup for simple communication by two chips
 * */
void nrf24_init(){
	Delay_mS(0.1);
	// EN_AA: this is done to have an automatic acknowledgement of data sent
	uint8_t val_one[5];
	val_one[0]=nrf24l01_EN_AA_ENAA_P0;//enable EN_AA on channel 1
	write_to_nrf(4,nrf24l01_EN_AA,val_one,1);
	
	//Pipe enabling 
	uint8_t val_two[5];
	val_two[0]=nrf24l01_EN_RXADDR_ERX_P0;
	write_to_nrf(4,nrf24l01_EN_RXADDR,val_two,1);//enable pipe 0
	
	//setup_address width
	uint8_t val_three[5];
	val_three[0]=nrf24l01_SETUP_AW_5BYTES;
	write_to_nrf(4,nrf24l01_SETUP_AW,val_three,1);//set channel width of 5 bytes
	
	//set up rf channel
	uint8_t val_four[5];
	val_four[0]=0x01;
	write_to_nrf(4,nrf24l01_RF_CH,val_four,1);//set channel frequency as 2.401Ghz
	
	//RF setup choosing the power mode and data speed
	uint8_t val_five[5];
	val_five[0]=nrf24l01_RF_SETUP_RF_PWR_0;//RF output power of 0dB 250kbps
	write_to_nrf(4,nrf24l01_RF_SETUP,val_five,1);// data rate of 1Mbs
	
	// Address setup
	//5 byte wide address
	//same address should be same if EN_AA is enabled
	uint8_t val_six[5];
	int i;
	for (i=0;i<5;i++){
		val_six[i]=0x12;
	}
	
	write_to_nrf(4,nrf24l01_RX_ADDR_P0,val_six,5);//receive address:5 bytes wide
	
	//TX_ADDR set the same address as RX forEN_AA
	int j;
	uint8_t val_seven[5];
		for (j=0;j<5;j++){
			val_seven[j]=0x12;
		}
		
		write_to_nrf(4,nrf24l01_TX_ADDR,val_seven,5);
	//Setting Max retries
		uint8_t val_eight[5];
		val_eight[0]=0x2F;
		write_to_nrf(4,nrf24l01_SETUP_RETR,val_eight,1);
		
	//Data Payload Width
	//determines the number of bytes you can send at a time
		uint8_t val_nine[5];
	val_nine[0]=5;
	write_to_nrf(4,nrf24l01_RX_PW_P0,val_nine,1);
	/*
	//activate feature
		val[0]=0x73;
		write_to_nrf(4,0x50,val,1);*/
	
	//FEATURE REGISTER
	//enable auto-acknowledgement
	uint8_t val_ten[5];
		val_ten[0]|=nrf24l01_EN_ACK_PAY;
		write_to_nrf(4,nrf24l01_FEATURE,val_ten,1);
		
	
		
	//CONFIG register setup
	//is responsible for powering up the module and setting whether it is a
	//receiver or transmitter
	//uint8_t transmitter,receiver;
	//transmitter=nrf24l01_CONFIG_PWR_UP|nrf24l01_CONFIG_CRCO|nrf24l01_CONFIG_EN_CRC||nrf24l01_CONFIG_MASK_MAX_RT;//|nrf24l01_CONFIG_MASK_MAX_RT
	//receiver=|=nrf24l01_CONFIG_PWR_UP|nrf24l01_CONFIG_CRCO|nrf24l01_CONFIG_EN_CRC|nrf24l01_CONFIG_MASK_MAX_RT|nrf24l01_CONFIG_PRIM_RX;
		uint8_t val_eleven[5];
		val_eleven[0]=0x32;
	write_to_nrf(4,nrf24l01_CONFIG,val_eleven,1);
		
	Delay_mS(0.1);
}
/*Function to transmit data. This is archieved by a high pulse on CE
 * which must wait for 10uS then low pulse to end transmission
 * */
void transmit_payload(uint8_t *w_buff){
	//first flush the TX buffer incase data is present
	write_to_nrf(4,nrf24l01_FLUSH_TX,w_buff,0);
	write_to_nrf(5,nrf24l01_W_TX_PAYLOAD,w_buff,5);//write payload to buffer
	Delay_mS(0.01);
	
	// Toggle CE to send the data on its way
	CE_ENABLE();
	Delay_mS(0.02);
	CE_DISABLE();
	Delay_mS(0.01);
	
}

/*Listening for data involved holding the CE pin High to listen out for data 
 * when the module is in receiver mode*/
void receive_payload(){
	CE_ENABLE();
	Delay_mS(1000);
	CE_DISABLE();
	
}
/*Reset the interrupt bits for the status register to transmit/receive
 * */
void reset(){
	Delay_mS(0.01);
	CSN_DISABLE();
	Delay_mS(0.01);
	uint8_t address;
	address=nrf24l01_W_REGISTER|nrf24l01_STATUS;
	spi_send_byte(address);
	Delay_mS(0.01);
	spi_send_byte(0x70);
	Delay_mS(0.01);
	CSN_ENABLE();
	
}

uint8_t get_status(){
	uint8_t *data;
	data=write_to_nrf(5,nrf24l01_STATUS,data,1);
	
	return data;
}
rgb_init(){
	//enable clock
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	//select pins 18 and 19 for red and green led
	PORTB_PCR18 = PORT_PCR_MUX(1)|PORT_PCR_DSE_MASK;  //Red led pin
	PORTB_PCR19 = PORT_PCR_MUX(1)|PORT_PCR_DSE_MASK;//Green led pin
	
	//choose the data direction
	GPIOB_PDDR |= RED_LED|GREEN_LED;
	//keeping the state of pins initially;keeps from lighting led 
	GPIOB_PSOR |= RED_LED;
	GPIOB_PSOR |= GREEN_LED;
}
