#ifndef _BOARD_H
#define _BOARD_H


#ifdef CUL_V3
#  define CC1100_CS_DDR		SPI_DDR
#  define CC1100_CS_PORT          SPI_PORT
#  define CC1100_CS_PIN		SPI_SS
#  define CC1100_IN_DDR		DDRD
#  define CC1100_IN_PORT          PIND
#  define CC1100_IN_PIN           PD2
#  define CC1100_OUT_DDR		DDRD
#  define CC1100_OUT_PORT         PORTD
#  define CC1100_OUT_PIN          PD3
#  define CC1100_INT		INT2
#  define CC1100_INTVECT          INT2_vect
#  define CC1100_ISC		ISC20
#else
#  define CC1100_CS_DDR		DDRC
#  define CC1100_CS_PORT          PORTC
#  define CC1100_CS_PIN		PC5
#  define CC1100_IN_DDR		DDRC
#  define CC1100_IN_PORT          PINC
#  define CC1100_IN_PIN           PC7
#  define CC1100_OUT_DDR		DDRC
#  define CC1100_OUT_PORT         PORTC
#  define CC1100_OUT_PIN          PC6
#  define CC1100_INT		INT4
#  define CC1100_INTVECT          INT4_vect
#  define CC1100_ISC		ISC40
#endif

#ifdef CUL_V3
#  define LED_DDR                 DDRE
#  define LED_PORT                PORTE
#  define LED_PIN                 6
#else
#  define LED_DDR                 DDRC
#  define LED_PORT                PORTC
#  define LED_PIN                 PC4
#endif


#define BOARD_ID_STR            "NHCUL868"
#define BOARD_ID_STR433         "NHCUL433"
#define BOARD_ID_USTR           L"NHCUL868"
#define BOARD_ID_USTR433        L"NHCUL433"

#define SPI_PORT		PORTB
#define SPI_DDR			DDRB
#define SPI_SS			PB0
#define SPI_MISO		PB3
#define SPI_MOSI		PB2
#define SPI_SCLK		PB1

#define HAS_USB                 1
#undef  HAS_FHT_8v              // PROGMEM:  434b, MEM: 19b
#undef HAS_FHT_80b             // PROGMEM: 1158b, MEM:  5b+FHTBUF_SIZE
#define FHTBUF_SIZE             48
#define FHTBUF_MODEL1           // see fht.c for details
#undef  FULL_CC1100_PA          //  100 byte PROGMEM
//#define RCV_BUCKETS             2       // 25byte per bucket
#define RCV_BUCKETS             0       // 25byte per bucket // stst
#define BUSWARE_CUL

#endif
