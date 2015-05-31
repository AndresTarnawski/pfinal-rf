/*
 * Prueba_Radio.c
 *
 * Created: 28/05/2015 15:28:03
 *  Author: Agustin
 */ 

#define F_CPU 16000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

#define _NOP()  asm("nop")
#define RF_BUFFER 128

void BoardInitialization(void);
void TransceiverInitialization(void);
void TransceiverByteTX(int);
int TransceiverRead(void);
void WDT_Disable(void);

volatile uint8_t channel = 0;

volatile int rssiRaw; // Global variable shared between RX ISRs
struct ringBuffer
{
	unsigned char buffer[127];
	volatile unsigned int head;
	volatile unsigned int tail;
} radioRXBuffer;

int main(void)
{
    BoardInitialization();
	TransceiverInitialization();
	int data = 0;
	while(1)
    {
        /******* Descomentar estas líneas para programar la placa como transmisora.******/
		//TransceiverByteTX(0x02);
		//_delay_ms(100);
		/********************************************************************************/
		
		/******** Comentar estas líneas para programar la placa como transmisora.********/
		data = TransceiverRead();	// Descomentar estas líneas para 
		if(data==0x02)
		{
			PORTB ^= 0x10;
			data = 0x00;
		}
		/********************************************************************************/
	}
}

void BoardInitialization(void)
{
	cli();
	WDT_Disable();
	DDRB = 0x10; //PB4 output
	PORTB = 0x10; //Turn off LED
	sei();
}

void TransceiverInitialization(void)
{
	for(int i=0; i<128; i++)
	{
		radioRXBuffer.buffer[i] = 0;
	}
	radioRXBuffer.tail = 0;
	radioRXBuffer.head = 0;
	
	/********************* Transceiver Pin Register - TRXPR *********************/
	// Este registro permite resetaer el transceiver sin resetear el MCU.
	
	TRXPR |= (1<<TRXRST); // Fuerza el reset del transceiver.
	
	/****************************************************************************/
	
	/*************** Transceiver Interrupt Enable Mask - IRQ_MASK ***************/
	// El transceiver admite hasta 8 interrupciones distintas.
	// Este registro habilita de manera individual cada una de ellas.
	
	IRQ_MASK = 0;  // Deshabilitamos por ahora todo tipo de interrupción.
	
	/****************************************************************************/
	
	/************** Transceiver State Control Register - TRX_STATE **************/
	// Este registro controla todos los estados del transceiver, a través
	// de los bits TRX_CMD. Como ni bien se enciende, apagamos el transceiver.
	
	TRX_STATE = (1 << TRX_CMD3);  // Set to TRX_OFF state
	
	/****************************************************************************/
	
	_delay_ms(5);
	
	/*************** Transceiver Control Register 1 - TRX_CTRL_1 ****************/
	// Contiene varias configuraciones. En particular, lo vamos a usar para
	// encender la verificación CRC de manera automática.
	TRX_CTRL_1 |= (1 << TX_AUTO_CRC_ON);  // Enable automatic CRC calculation.
	// Enable RX start/end and TX end interrupts
	IRQ_MASK = (1 << RX_START_EN)|(1 << RX_END_EN)|(1 << TX_END_EN);
	
	/****************************************************************************/

	/********* Transceiver Clear Channel Assessment (CCA) - PHY_CC_CCA **********/
	// Este registro se utiliza para configurar el canal. El modo CCA_MODE
	// debería estar configurado por default en modo Energy Above Threshold Mode.
	// El canal debería estar entre 11 y 26 (2405 MHz to 2480 MHz).
	
	if ((channel < 11) || (channel > 26))
	{
		channel = 11;	// Configuramos el canal 11.
		PHY_CC_CCA = (1 << CHANNEL4); // Configuramos el canal 11.
		PHY_CC_CCA &= ~ ((1 << CHANNEL3)|(1 << CHANNEL1));
	}
	/****************************************************************************/

	// Con toda la configuración lista, entramos a modo recepción.
	// Luego si nos interesa, cambiaremos esto en el modo transmisión.
	TRX_STATE &= ~ (1 << TRX_CMD3); // Salimos del estado TRX_OFF.
	TRX_STATE = ((1 << TRX_CMD2)|(1 << TRX_CMD1)); // Entramos al modo TRX_ON.
}

void TransceiverByteTX(int byte)
{
	int length = 3;

	// Transceiver State Control Register - TRX_STATE
	// This regiseter controls the states of the radio.
	// Set to the PLL_ON state - this state begins the TX.
	TRX_STATE &= ~ ((1 << TRX_CMD2)|(1 << TRX_CMD1)); // Salimos del modo TRX_ON.
	TRX_STATE = ((1 << TRX_CMD3)|(1 << TRX_CMD0));  // Entramos al modo PLL_ON.
	while((TRX_STATUS3)&(TRX_STATUS0))
	{
		_NOP();	// Esperamos a que se enganche el PLL.
	}

	// Start of frame buffer - TRXFBST
	// This is the first byte of the 128 byte frame. It should contain
	// the length of the transmission.
	TRXFBST = length;
	// Now copy the byte-to-send into the address directly after TRXFBST.
	memcpy((void *)(&TRXFBST+1), &byte, 1);

	// Transceiver Pin Register -- TRXPR.
	// From the PLL_ON state, setting SLPTR high will initiate the TX.
	TRXPR |= (1 << SLPTR);   // SLPTR = 1
	TRXPR &= ~ (1 << SLPTR);  // SLPTR = 0  // Then bring it back low
	
	// After sending the byte, set the radio back into the RX waiting state.
	TRX_STATE &= ~ ((1 << TRX_CMD3)|(1 << TRX_CMD0));  // Salimos del modo PLL_ON.
	TRX_STATE = ((1 << TRX_CMD2)|(1 << TRX_CMD1)); // Entramos al modo TRX_ON.
}

int TransceiverRead(void)
{
	if (!(radioRXBuffer.head == radioRXBuffer.tail))
	{
		// Read from the buffer tail, and update the tail pointer.
		char c = radioRXBuffer.buffer[radioRXBuffer.tail];
		radioRXBuffer.tail = (unsigned int)(radioRXBuffer.tail + 1) % RF_BUFFER;
		return c;
	}
	return 0;
}

void WDT_Disable(void) 
{
	// Disable watchdog timer
	asm("wdr");
	MCUSR = 0;
	WDTCSR |= (1 << WDCE) | (1 << WDE);
	WDTCSR = 0x00;
}

ISR(TRX24_TX_END_vect)
{
	PORTB ^= 0x10; // Acá se apaga y prende un LED cada vez que transmite.
}

ISR(TRX24_RX_END_vect)
{
	int length;
	// Maximum transmission is 128 bytes
	int tempFrame[RF_BUFFER];

	// The received signal must be above a certain threshold.
	if (rssiRaw & RX_CRC_VALID)
	{
		// The length of the message will be the first byte received.
		length = TST_RX_LENGTH;
		// The remaining bytes will be our received data.
		memcpy(&tempFrame[0], (void*)&TRXFBST, length);

		// Now we need to collect the frame into our receive buffer.
		//  k will be used to make sure we don't go above the length.
		//  i will make sure we don't overflow our buffer.
		unsigned int k = 0;
		unsigned int i = (radioRXBuffer.head + 1) % RF_BUFFER; // Read buffer head position and increment;
		while ((i != radioRXBuffer.tail) && (k < length-2))
		{
			// First, we update the buffer with the first byte in the frame
			radioRXBuffer.buffer[radioRXBuffer.head] = tempFrame[k++];
			radioRXBuffer.head = i; // Update the head
			i = (i + 1) % RF_BUFFER; // Increment i % BUFFER_SIZE
		}
	}
}

ISR(TRX24_RX_START_vect)
{
	rssiRaw = PHY_RSSI;  // Read in the received signal strength
}