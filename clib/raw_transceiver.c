/*
 * Copyright 2009 Stefan Strömberg
 *
 * This file is part of NetHomeServerCUL see <http://www.nethome.nu>.
 *
 * NetHomeServerCUL is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NetHomeServerCUL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NetHomeServer.  If not, see <http://www.gnu.org/licenses/>.
 *
 * This file is based on the work by R.Koenig
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/eeprom.h>
#include <util/parity.h>
#include <util/delay_basic.h>
#include <string.h>

#include "board.h"
#include "delay.h"
#include "raw_transceiver.h"
#include "led.h"
#include "cc1100.h"
#include "display.h"
#include "clock.h"
#include "fncollection.h"
#include "fht.h"
#include "cdc.h"

// End of message, must not be larger than 32767
#define SILENCE   30000


uint16_t credit_10ms;
uint8_t tx_report;              // global verbose / output-filter

static uint8_t cc_on;

#define QUEUE_MASK 0x07
#define QUEUE_LENGTH 8

#define STATE_IDLE_RX 0
#define STATE_BUSY_RX 1

#define SEND_BUFFER_SIZE 200

static uint16_t pulseLength = 0;
static uint8_t pulseLevel = 0;
static uint16_t pulseQueue[QUEUE_LENGTH];
static uint8_t readPointer = 0;
static uint8_t writePointer = 0;
static uint8_t state = STATE_IDLE_RX;

static uint8_t sendBuffer[SEND_BUFFER_SIZE];
static uint16_t sendBufferPointer = 0;
static uint16_t transmitPointer = 0;

/**
 * Initialize input/output pins for raw communication with CC1100.
 * Initialize internal data structures for receive and transmit
 */
void
initCC1100Com(void)
{
  CC1100_OUT_DDR  |=  _BV(CC1100_OUT_PIN);
  CC1100_OUT_PORT &= ~_BV(CC1100_OUT_PIN);
  CC1100_IN_DDR  &=  ~_BV(CC1100_IN_PIN);
  EICRB |= _BV(CC1100_ISC);                // Any edge of INTx generates an int.
  cc_on = 0;
}

/**
 * Set the tx_report parameter via the external command string
 */
void
setReportState(char *in)
{
  if(in[1] == 0) {              // Report Value
    DH(tx_report, 2); DNL();
    return;
  }
  fromhex(in+1, &tx_report, 1);
  restoreRadioState();
}

/**
 * Add a flank length to the send buffer
 */
void
addRawFlank(uint16_t flank) {
	// Check if it fits in one byte
	if (flank > (254 * 16)) {
		// No, signal two byte value by adding 255-byte
		sendBuffer[sendBufferPointer++] = 255;					// Mark long pulse
		sendBuffer[sendBufferPointer++] = flank >> (8 + 4);		// High byte
		sendBuffer[sendBufferPointer++] = (flank >> 4) & 0xFF;	// Lo Byte
	}
	else {
		// Flank fits in one byte
		sendBuffer[sendBufferPointer++] = (flank >> 4) & 0xFF;// One Byte
	}
}

/**
 * Get the length of the next flank in the send buffer
 */
uint16_t
getRawFlank(void) {
	uint16_t result = 0;

	// Check if it is a two byte value
	if (sendBuffer[transmitPointer] == 255) {
		// Yes, get the high byte first
		transmitPointer++;
		result = sendBuffer[transmitPointer++] << (8 + 4);
	}
	result += sendBuffer[transmitPointer++] << 4;
	return result;
}

/**
 * Add a new pulse to the transmit buffer.
 * \param in parameter string: "Ammmmssss" where "mmmm" is the mark flank length in us
 * in HEX format and "ssss" is the following space flank
 */
void
addRawPulse(char *in) {
	uint8_t pulseByte;
	uint16_t pulseWord;

	// NYI Bounds check
	if (sendBufferPointer >= SEND_BUFFER_SIZE) {
		DC('e');
		DNL();
		return;
	}

	// Add the Mark flank
	fromhex(in+1, &pulseByte, 1);					// Read high byte
	pulseWord = pulseByte << 8;
	fromhex(in+3, &pulseByte, 1);					// Read low byte
	pulseWord += pulseByte;
	addRawFlank(pulseWord);

	// Add the Space flank
	fromhex(in+5, &pulseByte, 1);					// Read high byte
	pulseWord = pulseByte << 8;
	fromhex(in+7, &pulseByte, 1);					// Read low byte
	pulseWord += pulseByte;
	addRawFlank(pulseWord);
	DC('o');
	DH(sendBufferPointer, 4);
	DNL();
}

/**
 * Transmit the content of the sendBuffer via RF
 */
void
sendRawMessage(char* in) {
	uint8_t repeat = 0;
	uint8_t repeatCount;
	transmitPointer = 0;
	uint8_t *t;
	uint16_t pulseLength;
	uint8_t onPeriod = 0;
	uint8_t offPeriod = 0;
	uint16_t repeatPointer = 0;
	uint8_t repeatPoint = 0;
	uint16_t pulseCounter = 0;

	// Get repeat count
	fromhex(in + 1, &repeat, 1);

	// Check if modulation period is specified
	if (in[3] != 0) {
		// Yes, it is. Read it.
		fromhex(in + 3, &onPeriod, 1);
		fromhex(in + 5, &offPeriod, 1);

		// Check if repeat offset is specified
		if (in[7] != 0) {
			fromhex(in + 7, &repeatPoint, 1);
		}
	}

	LED_ON();

	// To make absolutely sure that we do not transmit on any unauthorized frequencies,
	// we reset the oscillator frequency prior to transmission.
	// First check if it is 433.92 or 868.3 MHz
	t = EE_START_CC1100 + 0x0d;
	if (!bit_is_set(PINB, PB6)) {
		ewb(t++, 0x10);
		ewb(t++, 0xb0);
		ewb(t++, 0x71);
	} else {
		ewb(t++, 0x21);
		ewb(t++, 0x65);
		ewb(t++, 0xE8);
	}

	// Switch the CC1101 into transmit mode
	if(!cc_on) {
		initCC1100Com();
		ccInitChip();
		cc_on = 1;
	}
	ccStrobe( CC1100_SIDLE );
	my_delay_ms(1);
	ccTX();                       // Enable TX

	// Check if modulation was specified
	if (onPeriod > 0) {
		// Send the data from the transmit buffer "repeat" times where the mark pulses are modulated
		for (repeatCount = 0; repeatCount < repeat; repeatCount++) {
			pulseCounter = 0;
			for (transmitPointer = repeatPointer; transmitPointer < sendBufferPointer;) {
				// Since a pulse in the transmit buffer may take one or three bytes depending
				// on pulse length, we have to find on which transmit pointer position the
				// repeat point really is. The repeatCounter counts number of pulses so we
				// can find out when we have reached the repeat point and can save the
				// repeat pointer position
				if ((repeatCount == 0) && (pulseCounter == repeatPoint)) {
					repeatPointer = transmitPointer;
				}
				// Use the hardware counter for timing of the mark pulse
				// Reset hardware counter
				TCNT1 = 0;
				// Get mark pulse length
				pulseLength = getRawFlank();
				// Keep pulsing until hardware counter reaches pulseLength
				while (TCNT1 < pulseLength) {
					CC1100_OUT_PORT |= _BV(CC1100_OUT_PIN);         // High
					_delay_loop_1(onPeriod);
					CC1100_OUT_PORT &= ~_BV(CC1100_OUT_PIN);       // Low
					_delay_loop_1(offPeriod);
				}
				// Reset hardware counter an set output low
				TCNT1 = 0;
				CC1100_OUT_PORT &= ~_BV(CC1100_OUT_PIN);       // Low
				// Get space pulse length
				pulseLength = getRawFlank();
				// Keep low until hardware counter reaches pulseLength
				while (TCNT1 < pulseLength) {
					my_delay_us(2);
				}
				pulseCounter += 2;
			}
		}
	} else {
		// Send the data from the transmit buffer "repeat" times
		for (repeatCount = 0; repeatCount < repeat; repeatCount++) {
			pulseCounter = 0;
			for (transmitPointer = repeatPointer; transmitPointer < sendBufferPointer;) {
				// Since a pulse in the transmit buffer may take one or three bytes depending
				// on pulse length, we have to find on which transmit pointer position the
				// repeat point really is. The repeatCounter counts number of pulses so we
				// can find out when we have reached the repeat point and can save the
				// repeat pointer position
				if ((repeatCount == 0) && (pulseCounter == repeatPoint)) {
					repeatPointer = transmitPointer;
				}
				CC1100_OUT_PORT |= _BV(CC1100_OUT_PIN);         // High
				my_delay_us(getRawFlank());
				CC1100_OUT_PORT &= ~_BV(CC1100_OUT_PIN);       // Low
				my_delay_us(getRawFlank());
				pulseCounter += 2;
			}
		}
	}

	// Switch off Transmit mode
	if(tx_report) {               // Enable RX
		ccRX();
		// Temporary fix to disable reception interrupt
		if (tx_report == 0x03) {
			EIMSK  &= ~_BV(CC1100_INT);
		}
	} else {
		ccStrobe(CC1100_SIDLE);
	}
	LED_OFF();

	DC('o');
	DH(transmitPointer, 4);
	DNL();
}

/**
 * Reset send buffer write pointer
 */
void
resetSendBuffer(char* in) {
	// Reset sendPointer
	sendBufferPointer = 0;
	DC('o');
	DH(sendBufferPointer, 4);
}

/**
 * Read cc1101 configuration register
 */
void
readConfiguration(char* in) {
	uint8_t address = 0;
	uint8_t result;

	// Get address
	fromhex(in + 1, &address, 1);
	result = cc1100_read(address);
	DC('o');
	DH(result, 2);
}

/**
 * Write cc1101 configuration register
 */
void
writeConfiguration(char* in) {
	uint8_t address = 0;
	uint8_t data;
	uint8_t result;

	// Get address
	fromhex(in + 1, &address, 1);

	// Get data
	fromhex(in + 3, &data, 1);

	result = cc1100_write(address, data);
	DC('o');
}

/**
 * Reception report task. Looks for new received pulses in the in-queue and reports them
 * over the USB-port
 */
TASK(ReportTask)
{
	if (readPointer != writePointer) {
		readPointer = (readPointer + 1) & QUEUE_MASK;
		if (pulseQueue[readPointer] & 0x8000) {
			DC('m');
			// Retrigger timeout timer. This is triggered on mark pulse, otherwise the
			// last "faked" space pulse would constantly retrigger the counter
			OCR1A = SILENCE;
			TIMSK1 = _BV(OCIE1A);             // On timeout analyze the data
			TIFR1 = _BV(OCF1A);
		}
		else {
			DC('s');
		}
		pulseLevel = 0; // Reset for next pulse
		DH((pulseQueue[readPointer] >> 8) & 0x7F, 2);
		DH(pulseQueue[readPointer] & 0xFF, 2);
		DNL();
	}
}

/**
 * Timer compare interrupt. This is called when there has been SILENCE time since last pulse
 * flank was received. Send the "end" pulse and go to idle state
 */
ISR(TIMER1_COMPA_vect)
{
	uint8_t newWritePointer;

	TIMSK1 = 0;                           // Disable "us"

	// Signal this as a low pulse
	// Update write pointer
	newWritePointer = (writePointer + 1) & QUEUE_MASK;
	if (newWritePointer == readPointer) {
		// Overflow!
		newWritePointer = writePointer;
	}

	// Write pulse to queue
	pulseQueue[newWritePointer] = SILENCE;
	writePointer = newWritePointer;

	// We are now in idle state
	state = STATE_IDLE_RX;

	// Take printing out of bulk mode
	cdc_bulk_mode = 0;
}

/**
 * Flank detected-interrupt. Measure the pulse length and report it
 */
ISR(CC1100_INTVECT)
{
	uint8_t newWritePointer;

	// Grab counter value fast and restart counter
	pulseLength = TCNT1;
	TCNT1 = 0;

	// Check our state, if we are not in busy, ignore this signal but go to busy
	if (state == STATE_BUSY_RX) {
		// Update write pointer
		newWritePointer = (writePointer + 1) & QUEUE_MASK;
		if (newWritePointer == readPointer) {
			// Overflow!
			newWritePointer = writePointer;
			pulseLength = 0x7FFF;
		}

		// Use highest bit to signal mark or space
		if (!bit_is_set(CC1100_IN_PORT,CC1100_IN_PIN)) {
			pulseLength |= 0x8000;
		}

		// Write pulse to queue
		pulseQueue[newWritePointer] = pulseLength;
		writePointer = newWritePointer;
	}
	// Since we got a signal, we are in busy state
	state = STATE_BUSY_RX;

	// Put printing in bulk mode to fill up USB-buffer before sending
	cdc_bulk_mode = 1;
}

/**
 * Switch off CC1100 radio (both for TX and RX)
 */
void
radioOff(void)
{
  ccStrobe(CC1100_SIDLE); // Turn off frequency synthesizer
  ccStrobe(CC1100_SPWD);  // Enter power down mode
  cc_on = 0;
}

/**
 * Switch on the CC1100 radio in reception mode
 */
void
radioOnRx(void)
{
  ccInitChip();
  cc_on = 1;
  ccRX();
}

/**
 * Setup reception mode according to the tx_report parameter
 */
void
restoreRadioState()
{
  initCC1100Com();
  if(tx_report) {
    radioOnRx();
  } else {
    radioOff();
  }
  // Temporary fix to disable reception interrupt
  if (tx_report == 0x03) {
	  EIMSK  &= ~_BV(CC1100_INT);
  }
}

