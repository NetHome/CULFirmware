#include <avr/io.h>
#include "board.h"
#include "display.h"
#include "delay.h"
#ifdef HAS_LCD
#include "pcf8833.h"
#endif
#include <MyUSB/Drivers/USB/USB.h>     // USB_IsConnected

uint8_t battery_state;

//*****************************************************************************
// Function: get_adcw
// Parameters: none.
// Returns: none.
//
// Description: start AD conversion to read specific ADC channel
//*****************************************************************************

uint16_t
get_adcw(uint8_t mux)
{
  uint8_t i;
  uint16_t result = 0;
  
  // Frequenzvorteiler setzen auf 8 (1) und ADC aktivieren (1)
  ADCSRA = (1<<ADEN) | (1<<ADPS1) | (1<<ADPS0); 

  ADMUX = mux;                      // Kanal waehlen
  ADMUX |= (1<<REFS1) | (1<<REFS0); // interne Referenzspannung nutzen

  for(i=0; i<2; i++) {
    ADCSRA |= (1<<ADSC);              // Dummy readout
    while ( ADCSRA & (1<<ADSC) );     // wait for converter to finish
  }
  my_delay_ms(1);
  
  /* Eigentliche Messung - Mittelwert aus 4 aufeinanderfolgenden Wandlungen */
  for(i=0; i<4; i++) {
    ADCSRA |= (1<<ADSC);            // eine Wandlung "single conversion"
    while ( ADCSRA & (1<<ADSC) );   // auf Abschluss der Konvertierung warten
    result += ADCW;                 // Wandlungsergebnisse aufaddieren
  }
  ADCSRA &= ~(1<<ADEN);             // ADC deaktivieren (2)
  
  result /= 4; 
  
  return result;
}

static void
raw2percent(uint16_t bv)
{
  static uint8_t bat[] = { 0,109,136,141,143,152,164,179,196,214,240 };
  uint8_t s1, s2;

  if(bv < 761) bv = 761;
  bv -= 760;
  if(bv > 240) bv = 240;

  uint8_t b = (uint8_t)bv;
  for(s1 = 1; s1 < sizeof(bat); s1++)
    if(b <= bat[s1])
      break;
  
  s2 = s1-1;
  battery_state = 10*s2+(10*(uint16_t)(b-bat[s2]))/(uint16_t)(bat[s1]-bat[s2]);
}

void
batfunc(char *in)
{
  uint8_t hb[2];
  uint16_t bv;
  uint8_t l = fromhex(in+1,hb,2);

#ifdef CURV3

  if(l == 2) {
    uint8_t pin = BAT_PEN2;
    if(hb[0] == 1) pin = BAT_USUS;
    if(hb[1]) {
      BAT_PORT |= _BV(pin);
    } else {
      BAT_PORT &= ~_BV(pin);
    }

  } else
#endif
  {
    // This will displayed on the 13char title row on the CUR
    bv = get_adcw(BAT_MUX);
    if(l >= 1 && hb[0]) {
      DU(bv, 4);

    } else {
      raw2percent(bv);
      DU(battery_state,3);
      DC('%');

    }
#ifdef CURV3
    if( !bit_is_set( BAT_PIN, BAT_CHG ) ) DS_P( PSTR(" charging"));
    if( !bit_is_set( BAT_PIN, BAT_DONE) ) DS_P( PSTR(" charged"));
    if( !bit_is_set( BAT_PIN, BAT_FLT)  ) DS_P( PSTR(" error"));
#else
    uint8_t s1;
    s1  = (bit_is_set( BAT_PIN, BAT_PIN1) ? 2 : 0);
    s1 |= (bit_is_set( BAT_PIN, BAT_PIN2) ? 1 : 0);
    if(s1==0) DS_P( PSTR(" discharge"));
    if(s1==1) DS_P( PSTR(" charged"));
    if(s1==2) DS_P( PSTR(" charging"));
    if(s1==3) DS_P( PSTR(" error"));
#endif

#ifdef CURV3
    if( !bit_is_set( BAT_PIN, BAT_DOK) ) DS_P( PSTR(" DC"));   // not wired ;)
    if( !bit_is_set( BAT_PIN, BAT_UOK) ) DS_P( PSTR(" USB"));
#endif
    DNL();
  }
}


#if defined(HAS_LCD) && defined(BAT_PIN)
void
bat_drawstate(void)
{
  if(!lcd_on)
    return;

  raw2percent(get_adcw(BAT_MUX));
  uint16_t v1 = battery_state*VISIBLE_WIDTH/100;

  lcd_line(WINDOW_LEFT,TITLE_HEIGHT-1,          // the green/blue part
                   WINDOW_LEFT+v1,TITLE_HEIGHT-1, 0x0f00);

#ifdef CURV3
  lcd_line(WINDOW_LEFT+v1,TITLE_HEIGHT-1,       // and the red part
           WINDOW_RIGHT,TITLE_HEIGHT-1,
           bit_is_set( PINA, PA4 ) ? 0xf000 : 0xf0);  // charging:blue else red
#else
  uint8_t s1;
  s1  = (bit_is_set( BAT_PIN, BAT_PIN1) ? 2 : 0);
  s1 |= (bit_is_set( BAT_PIN, BAT_PIN2) ? 1 : 0);
  lcd_line(WINDOW_LEFT+v1,TITLE_HEIGHT-1,       // and the red part
           WINDOW_RIGHT,TITLE_HEIGHT-1,
           (s1==2 && USB_IsConnected) ? 0xf0 : 0xf000);// charging:blue else red
#endif

}
#endif

void
bat_init(void)
{
#ifdef CURV3
     BAT_DDR  = 0xE;    // BAT_CEN BAT_PEN2 BAT_USUS
     BAT_PORT = 0xf1 | _BV( BAT_PEN2 ); // Pull and charge 500mA
#else
     BAT_DDR  &= ~(_BV(BAT_PIN1) | _BV(BAT_PIN1));
     BAT_PORT |= (_BV(BAT_PIN1) | _BV(BAT_PIN1));          // pull Battery stati
#endif
}
