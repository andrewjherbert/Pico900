// Elliott 900 emulator for Raspberry Pi Pico

// Copyright (c) Andrew Herbert - 03/04/2021

// MIT Licence.

// Emulator for Elliott 900 Series computers.
// Does not implement 'undefined' effects.
// Has simplified handling of priority levels and initial orders.
// Only supports paper tape input and output and teleprinter
// peripherals.


/**********************************************************/
/*                     HEADER FILES                       */
/**********************************************************/


#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <pico/binary_info.h>


/**********************************************************/
/*                         TYPES                          */
/**********************************************************/

// Each 900 18 bit word is stored as a 32 bit unsigned value.
// For some operations we compute with a double word so we
// use 64 bit unsigned quantities for these. Arithmetic is
// performed using 32 and 64 bit integers.

// Address calculations 16 bit unsigned arithmetic is used.

// Input/output uses 8 bit characters.

typedef int_fast32_t   INT32;
typedef int_fast64_t   INT64;
typedef uint_fast8_t   UINT8;
typedef uint_fast16_t  UINT16;
typedef uint_fast32_t  UINT32;
typedef uint_fast64_t  UINT64;


/**********************************************************/
/*                         DEFINES                        */
/**********************************************************/


// Booleans
#define TRUE  1
#define FALSE 0

// Store size
#define STORE_SIZE 8192

// GPIO pins

#define RDR_1_PIN    0 // Set to lsb of reader input
#define RDR_2_PIN    1 // these pins are assumed consecutive
#define RDR_4_PIN    2
#define RDR_8_PIN    3
#define RDR_16_PIN   4
#define RDR_32_PIN   5
#define RDR_64_PIN   6
#define RDR_128_PIN  7 // Set to msb of reader input

#define RDR_MASK     0377 // PINs to bit mask

#define PUN_1_PIN    8 // Pico sets to lsb of punch output
#define PUN_2_PIN    9 // 
#define PUN_4_PIN   10
#define PUN_8_PIN   11
#define PUN_16_PIN  12
#define PUN_32_PIN  13
#define PUN_64_PIN  14 
#define PUN_128_PIN 15 // Pico sets to msb of punch output

#define PUN_MASK    0177400 // PINs to bit mask

#define NOPOWER_PIN 16 // set HIGH to stop and reset, set LOW to run
#define ACK_PIN     17 // pulse HIGH to acknowledge RDR or PUN request
#define II_AUTO_PIN 18 // set HIGH to autostart on reset, or LOW to execute
                       // initial instructions
#define LOG_PIN     19 // set HIGH to enable logging to USB serial port
#define FAST_PIN    20 // set HIGH to enable running at full speed
#define RDRREQ_PIN  21 // Pico sets HIGH to request reader input and awaits ACK
#define PUNREQ_PIN  22 // Pico sets HIGH to request punch output and awiats ACK
#define TTYSEL_PIN  26 // Pico sets HIGH to select teleprinter, LOW for paper tape
// GPIO24 spare
#define LED_PIN     25 // onboard LED
// GPIO26 spare
// GPIO27 spare
// GPIO28 spare

#define IN_PINS  13 // count of input pins
#define OUT_PINS 12 // count of output pins

// Useful constants
#define BIT_19    01000000
#define MASK_18   0777777
#define BIT_18    00400000
#define MASK_16   00177777
#define ADDR_MASK 8191
#define MOD_MASK  00160000
#define MOD_SHIFT 13
#define FN_MASK   15
#define FN_SHIFT  13

// Locations of B register and SCR for priority levels 1 and 4
#define SCR_LEVEL_1   0
#define SCR_LEVEL_4   6
#define B_REG_LEVEL_1 1
#define B_REG_LEVEL_4 7

// Codes for emulation failures
#define EMULATION_SCR_FAIL 3
#define EMULATION_14_FAIL  4
#define EMULATION_15_FAIL  5


/**********************************************************/
/*                         GLOBALS                        */
/**********************************************************/

UINT32 store[STORE_SIZE]; // core store

const UINT8 in_pins[IN_PINS] =  { NOPOWER_PIN, ACK_PIN, II_AUTO_PIN, LOG_PIN, FAST_PIN,
		                  RDR_1_PIN, RDR_2_PIN, RDR_4_PIN, RDR_8_PIN, RDR_16_PIN,
		                  RDR_32_PIN, RDR_64_PIN, RDR_128_PIN };

const UINT8 out_pins[OUT_PINS] = { RDRREQ_PIN, PUNREQ_PIN, TTYSEL_PIN, PUN_1_PIN, PUN_2_PIN,
			           PUN_4_PIN, PUN_8_PIN, PUN_16_PIN, PUN_32_PIN, PUN_64_PIN,
			           PUN_128_PIN, LED_PIN };


/**********************************************************/
/*                         FUNCTIONS                      */
/**********************************************************/


void   wait_for_power();             // Wait for NOPOWER = 0
void   wait_for_no_power();          // Wait for NOPOWER = 1
void   emulate(const UINT16 start, UINT8 fast);  // run emulation from start
void   clear_store();                // clear main store
UINT8  read_tape();                  // read from paper tape
void   punch_tape(const UINT8 ch);   // punch to paper tape
UINT8  read_tty();                   // read from teletype
void   write_tty(const UINT8 ch);    // write to teletype
void   load_initial_instructions();   // load initial orders
UINT32 make_instruction(const UINT8 m, const UINT8 f, const UINT16 a);
                                     // helper for load_initial_instructions
void   set_up_gpios();               // initial GPIO interface to outside world
void   slow_blink();                 // signal waiting for NOPOWER = 0
void   steady_glow();                // signal running
void   emulation_fail(const char *msg, const UINT8 code); // e.g., illegal instruction
                                     // encountered
UINT32 no_power();                   // status of NOPOWER
void   wait_for_power();             // wait for NOPOWER LOW
void   wait_for_no_power();          // wait for NOPOWER HIGH
void   wait_for_ack();               // wait for ACK HIGH
void   wait_for_no_ack();            // wait for ACK LOW
UINT8  get_reader_ch();              // read from paper tape reader
UINT8  get_tty_ch();                 // read from teletype
void   put_punch_ch(UINT8 ch);       // punch to paper tape punch
void   put_tty_ch(UINT8 ch);         // punch to teleprinter


/**********************************************************/
/*                           MAIN                         */
/**********************************************************/

int main () {

  bi_decl(bi_program_description("Elliott 920M Emulator by Andrew Herbert"));

  stdio_init_all(); // initialise Pico libraries

  set_up_gpios(); // configure interface to outside world

  while (TRUE) // run emulation for ever
    {
      if ( gpio_get(LOG_PIN) ) puts("PICO900 starting");
      wait_for_power(); // wait for external world to signal a reset
      steady_glow(); // set LED to indicate running
      clear_store();
      load_initial_instructions(); // load initial instructions into store
      emulate(8181, gpio_get(FAST_PIN)); // run emulation starting at location 8181
      // will only end up here on a failure or reset, in either case, restart
    }
}


/**********************************************************/
/*                         EMULATION                      */
/**********************************************************/


void emulate (const UINT32 start, UINT8 fast) {

  static UINT32 a_reg  = 0;
  static UINT32 q_reg  = 0;
  static UINT32 b_reg  = B_REG_LEVEL_1;
  static UINT32 sc_reg = SCR_LEVEL_1; // address in store of B static and SCR
  static UINT8  level = 1; // priority level
  static UINT32 instruction, f, a, m;
  static INT32  last_scr = -1; // scr of previous instruction

  store[sc_reg] = start; // execution begins at location 'start'.

  // instruction fetch and decode loop
  while ( TRUE )
    {
      if ( no_power() ) return; // reset signalled

      if (!fast) sleep_us(11); // run at roughly 920M speed

      // increment SCR
      last_scr = store[sc_reg]++; // remember SCR
      if   ( last_scr >= STORE_SIZE )
        {
          emulation_fail("*** SCR has overflowed the store\n",
			 EMULATION_SCR_FAIL);
 	  /* NOT REACHED */
        }

      // fetch and decode instruction;
      instruction = store[last_scr];
      f = (instruction >> FN_SHIFT) & FN_MASK;
      a = (instruction & ADDR_MASK) | (last_scr & MOD_MASK);

      // perform B modification if needed
      if ( instruction >= BIT_18 )
        {
  	  m = (a + store[b_reg]) & MASK_16;
	}
      else
	  m = a & MASK_16;

      // perform function determined by function code f
      switch ( f )
        {

        case 0: // Load B
	    q_reg = store[m]; store[b_reg] = q_reg;
	    break;

          case 1: // Add
       	    a_reg = (a_reg + store[m]) & MASK_18;
	    break;

          case 2: // Negate and add
	    a_reg = (UINT32) ((INT32) store[m] - (INT32) a_reg) & MASK_18;
	    break;

          case 3: // Store Q
	    store[m] = q_reg >> 1;
	    break;

          case 4: // Load A
	    a_reg = store[m];
	    break;

          case 5: // Store A
	    if   ( level == 1 && m >= 8180 && m <= 8191 )
	      {
		break; // ignore write to II at level 1
	      }
	    else
	      store[m] = a_reg; 
	    break;

          case 6: // Collate
	    a_reg &= store[m];
	    break;

          case 7: // Jump if zero
  	    if   ( a_reg == 0 ) store[sc_reg] = m;
	    break;

          case 8: // Jump unconditional
	    store[sc_reg] = m;
	    break;

          case 9: // Jump if negative
	    if   ( a_reg >= BIT_18 ) store[sc_reg] = m;
	    break;

          case 10: // increment in store
 	    store[m] = (store[m] + 1) & MASK_18;
	    break;

          case 11:  // Store S
	    {
	      q_reg = store[sc_reg] & MOD_MASK;
	      store[m] = store[sc_reg] & ADDR_MASK;
	      break;
	    }

          case 12:  // Multiply
	    {
	      // extend sign bits for a and store[m]
	      const INT64 al = ( ( a_reg >= BIT_18 )
				 ? ((INT64) a_reg - (INT64) BIT_19)
				 : (INT64) a_reg );
	      const INT64 sl = ( ( store[m] >= BIT_18 )
				 ? store[m] - BIT_19
				 : store[m] );
	      INT64 prod = al * sl;
	      q_reg = (UINT32) ((prod << 1) & MASK_18);
	      if   ( al < 0 ) q_reg |= 1;
	      prod = prod >> 17; // arithmetic shift
 	      a_reg = (UINT32) (prod & MASK_18);
	      break;
	    }

          case 13:  // Divide
	    {
	      // extend sign bit for aq
	      const INT64 al   = ( ( a_reg >= BIT_18 )
				   ? (INT64) a_reg - (INT64) BIT_19
				   : (INT64) a_reg ); // sign extend
	      const INT64 ql   = (INT64) q_reg;
	      const INT64 aql  = (al << 18) | ql;
	      const INT64 sl   = ( ( store[m] >= BIT_18 )
				   ? (INT64) store[m] - (INT64) BIT_19
				   : (INT64) store[m] );
              const INT32 quot = (INT32)(( aql / sl) >> 1) & MASK_18;
  	      a_reg = quot | 1;
	      q_reg = quot & 0777776;
	      break;
	    }

          case 14:  // Shift - assumes >> applied to a signed INT64 or INT32
	            // is arithmetic
	    {
              UINT8 places = m & ADDR_MASK;
	      const UINT64 al  = ( ( a_reg >= BIT_18 )
				   ? (INT64) a_reg - (INT64) BIT_19
				   : (INT64) a_reg ); // sign extend
	      const UINT64 ql  = q_reg;
	      UINT64 aql = (al << 18) | ql;
	      
	      if   ( places <= 2047 )
	        {
	          if   ( places >= 36 ) places = 36;
	          aql <<= places;
	        }
	      else if ( places >= 6144 )
	        { // right shift is arithmetic
	          places = 8192 - places;
	          if ( places >= 36 ) places = 36;
		  aql >>= places;
	        }  
	      else
	        {
	          emulation_fail("*** Unsupported i/o 14 i/o instruction\n",
				 EMULATION_14_FAIL);
	          /* NOT REACHED */
	        }

	      q_reg = (INT32) (aql & MASK_18);
	      a_reg = (INT32) ((aql >> 18) & MASK_18);
	      break;
	    }

            case 15:  // Input/output etc
	      {
                const UINT16 z = m & ADDR_MASK;
	        switch   ( z )
	    	  {

		    case 2048: // read from tape reader
		      { 
	                UINT8 ch = read_tape(); 
	                a_reg = ((a_reg << 7) | ch) & MASK_18;
	                break;
	               }

	            case 2052: // read from teletype
		      {
	                UINT8 ch = read_tty();
	                a_reg = ((a_reg << 7) | ch) & MASK_18;
	                break;
	              }

	            case 6144: // write to paper tape punch 
	              punch_tape((UINT8)(a_reg & 255));
	              break;

	            case 6148: // write to teletype
	              write_tty((UINT8)(a_reg & 255));
	              break;	      
	  
	            case 7168:  // Level terminate
	              level = 4;
	              sc_reg = SCR_LEVEL_4;
		      b_reg  = B_REG_LEVEL_4;
	              break;

	            default:
		      emulation_fail("*** Unsupported 15 i/o instruction\n",
				     EMULATION_15_FAIL);
	              /* NOT REACHED */
		  } // end 15 switch
	      } // end case 15
	} // end function switch
    } // end while fetching and decoding instructions
}

void clear_store() {
  for ( UINT16 i=0 ; i < STORE_SIZE ; ++i ) store[i] = 0;
}

/**********************************************************/
/*                    PAPER TAPE SYSTEM                   */
/**********************************************************/


/* Paper tape reader */
UINT8 read_tape() {
  if ( gpio_get(LOG_PIN) ) puts("Read from paper tape\n");
  return get_reader_ch();  }

/* paper tape punch */
void punch_tape(const UINT8 ch) {
  if ( gpio_get(LOG_PIN) ) puts("Punch to paper tape\n");
  put_punch_ch(ch);
}

/* Teletype */
UINT8 read_tty() {
  if ( gpio_get(LOG_PIN) ) printf("Read from teleprinter\n");
  return get_reader_ch();
}

void write_tty(const UINT8 ch) {
  if ( gpio_get(LOG_PIN) ) printf("Punch to teleprinter (%d)\n", ch);
  put_tty_ch(ch);
}


/**********************************************************/
/*               INITIAL INSTRUCTIONS                     */
/**********************************************************/


void load_initial_instructions() {
  store[8180] = (-3 & MASK_18);
  store[8181] = make_instruction(0,  0, 8180);
  store[8182] = make_instruction(0,  4, 8189);
  store[8183] = make_instruction(0, 15, 2048);
  store[8184] = make_instruction(0,  9, 8186);
  store[8185] = make_instruction(0,  8, 8183);
  store[8186] = make_instruction(0, 15, 2048);
  store[8187] = make_instruction(1,  5, 8180);
  store[8188] = make_instruction(0, 10,    1);
  store[8189] = make_instruction(0,  4,    1);
  store[8190] = make_instruction(0,  9, 8182);
  store[8191] = make_instruction(0,  8, 8177);
  printf("Initial orders loaded\n");
}

UINT32 make_instruction(const UINT8 m, const UINT8 f, const UINT16 a) {
  return (((UINT32)m << 17) | ((UINT32)f << 13) | (UINT32)a);
}

/**********************************************************/
/*                         PICO GPIO                      */
/**********************************************************/


void set_up_gpios() {
  UINT32 in_pins_mask = 0, out_pins_mask = 0;
  for ( UINT8 i = 0; i < IN_PINS;  i++ ) in_pins_mask  |= (1 << i);
  for ( UINT8 i = 0; i < OUT_PINS; i++ ) out_pins_mask |= (1 << i);

  // set up GPIO directions
  gpio_set_dir_all_bits(out_pins_mask); // spare bits set to be inputs

  // Set pull up / downs
  for ( UINT8 i =  0 ; i < IN_PINS; i++ )
    {v
      UINT8 pin = in_pins[i];
      if  ( pin != NOPOWER_PIN )
	gpio_pull_down(pin);
      else
	gpio_pull_up(pin);
    }
}

void fast_blink() {
  gpio_put(LED_PIN, 0);
  sleep_ms(250);
  gpio_put(LED_PIN, 1);
  sleep_ms(250);
}
void slow_blink() {
  gpio_put(LED_PIN, 0);
  sleep_ms(250);
  gpio_put(LED_PIN, 1);
  sleep_ms(1000);
}

void steady_glow() {
  gpio_put(LED_PIN, 1);
}

void emulation_fail(const char* msg, const UINT8 code) {
  // flash code then pause
  static UINT8 count = 0;
  while ( TRUE ) {
    if ( gpio_get(LOG_PIN) && ((++count % 5) == 0) )
      {
	count = 0;
	printf("Emulation failed: %s\n", msg);
      }
    for ( int i = 1 ; i <= code ; i++ ) fast_blink();
    sleep_ms(2000);
  }
}
    
UINT32 no_power() {
  return gpio_get(NOPOWER_PIN);
}

void wait_for_power() {
  static UINT8 count = 0;
  while ( no_power() ) {
    if ( gpio_get(LOG_PIN) && ((++count % 5) == 0) )
      {
	count = 0;
	puts("Waiting for NOPOWER to go to 0\n");
      }
    slow_blink();
  }
}

void wait_for_no_power() {
  static UINT8 count = 0;
  while ( !no_power() ) {
    if ( gpio_get(LOG_PIN) && ((++count % 5) == 0) )
      {
	count = 0;
	puts("Waiting for NOPOWER to go to 1\n");
      }
    sleep_us(100);
  }
}

UINT8 get_reader_ch() {
  static UINT8 ch;
  gpio_put(RDRREQ_PIN, 1);
  wait_for_ack();
  ch = (gpio_get_all() & RDR_MASK) >> RDR_1_PIN; // read 8 bits
  wait_for_no_ack();
  gpio_put(RDRREQ_PIN, 0);
  return ch;
}

UINT8 get_tty_ch() {
  static UINT8 ch;
  gpio_put(TTYSEL_PIN, 1);
  gpio_put(RDRREQ_PIN, 1);
  wait_for_ack();
  ch = (gpio_get_all() & RDR_MASK) >> RDR_1_PIN;
  wait_for_no_ack();
  gpio_put(RDRREQ_PIN, 0);
  gpio_put(TTYSEL_PIN, 0);
  return ch;
}

void put_punch_ch(const UINT8 ch) {
  gpio_put_masked(PUN_MASK, ch << PUN_1_PIN); // write 8 bits
  gpio_put(PUNREQ_PIN, 1);
  wait_for_ack();
  wait_for_no_ack();
  gpio_put(PUNREQ_PIN, 0);
}

void put_tty_ch(const UINT8 ch) {
  gpio_put_masked(PUN_MASK, ch << PUN_1_PIN); // write 8 bits
  gpio_put(TTYSEL_PIN, 1);
  gpio_put(PUNREQ_PIN, 1);
  wait_for_ack();
  wait_for_no_ack();
  gpio_put(PUNREQ_PIN, 0);
  gpio_put(TTYSEL_PIN, 0);
}

void wait_for_ack() {
  static UINT8 count = 0;
  while ( !gpio_get(ACK_PIN) ) {
    if ( gpio_get(LOG_PIN) && ((++count % 5) == 0) )
      {
	count = 0;
	puts("Waiting for ACK to go to 1\n");
      }
    slow_blink();
    if ( no_power() ) return; // a reset has been signalled
  }
}

void wait_for_no_ack() {
  static UINT8 count = 0;
  while ( gpio_get(ACK_PIN) ) {
    if ( gpio_get(LOG_PIN) && ((++count % 5) == 0) )
      {
	count = 0;
	puts("Waiting for ACK to go to 0\n");
      }
    slow_blink();
    if ( no_power() ) return; // a reset has been signalled
  }
}

