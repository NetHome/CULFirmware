#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include "mysleep.h"
#include "display.h"
#include "pcf8833.h"
#include "joy.h"
#include "led.h"
#include "battery.h"
#include "transceiver.h"
#include "fncollection.h"


uint8_t sleep_time;

void
mysleep(char *in)
{
  fromhex(in+1, &sleep_time, 1);
  ewb(EE_SLEEPTIME, sleep_time);
  DS_P( PSTR("Sleep: ") );
  DU(sleep_time, 2);
  DNL();
}

void
dosleep(void)
{
  LED_OFF();
  wdt_disable(); 
  joy_enable_interrupts();
  lcd_switch(0);
  radioOff();


  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  sleep_enable();
  sei();
  sleep_cpu();


  // Wake-up sequence
  sleep_disable();
  joy_disable_interrupts();
  sei();
  wdt_enable(WDTO_2S); 
  lcd_switch(1);
  bat_drawstate();
  restoreRadioState();      // One is somehow not enough.
  restoreRadioState();
}
