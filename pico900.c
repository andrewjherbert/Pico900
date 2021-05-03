// Elliott 900 emulator for Raspberry Pi Pico
// Copyright (c) Andrew Herbert - 28/04/2021

// MIT Licence.

// Emulator for Elliott 900 Series computers.
// Does not implement 'undefined' effects.
// Has simplified handling of priority levels and initial orders.
// Only supports paper tape input and output and teleprinter
// peripherals.

// This program is intended to run on a Raspberry Pi Pico and
// depends upon Pico specific functions and data types.

// int_fast / uint_fast types are used to ensure data are held
// in appropriate length words and registers of the Pico. The 900
// series are 18 bit machines, so 900 series store  and registers
// are held in 32 bit variables.  In the multiply (12) and divide
// (13) instructions calculations are performed using the A and Q
// registers as a 36 bit quantity and so the calculations for
// these two instructions use 64 bit quantities.  The results of
// each instruction, whether in a register or a store location
// are held masked to 18 bits. Thus the test for a negative number
// in a register or store location is to check if it's bit 18
// is set.  Addresses are generally hed as unsigned 16 bit
// quantities and characters as unsigned 8 bit quantities, but on
// the Pico both are implmented as 32 bit quantities so there is
// no saving of memory, just a reminder of the size of these types
// of data.

// The Pico emulates the paper tape station  interface of a 920M 
// using GPIO pins.  The pin numbers are listed below.  The use 
// of eachpin is as follows:
//
// RDRReq_PIN, high signals a reader request.  The paper tape
// station is expected to load 8 bits of data on pins RDR_1_PIN
// (lsb) to  RDR_128_PIN (msb) and then raise ACK-PIN high for
// approximately 5uS to indicate the input data is ready. Once
// the computer has read in the data it lowers RDRReq-PIN to
// signal transfer complete.  
//
// PUNReq_PIN, high signals a punch request. The paper tape
// station is expected to laod 8 bits of data from pins PUN_1_PIN
// (lsb) to PUN_128_PIN (msb) and then raise ACK_PIN high for
// approximately 5uS to indicate the output data have been copied.
// Once the ACK_PIN is lowered the computer then lowers PUNReq-PIN.
//
// TTYSEL_PIN, high signals that paper tape input and output is
// to be directed to the online teleprinter, if present. Low
// signals that input should come from the paper tape reader
// and output to the paper tape punch.
//
// ACK_PIN, used as described above to signal completion of a
// data transfer to/from the paper tape station.
//
// II_AUTO_PIN, high selects that on a reset / restart the
// computer should execute an autostart, i.e., jump to location
// 8177.  If low the computer should obey the initial orders
// to load in a program from paper tape.
//
// NOPOWER_PIN, high signals that the computer should stop
// execution and enter a reset state.  Low signals that
// the computer whould wake up if in the reset state and
// execute an autostart or initial instructions as determined
// by II_AUTO.  NOPOWER should remain low so that the computer
// can continue to execute until next reset by raising NOPOWER
// high again.
//
// Two additional GPIO pins are used to set options for the
// emulation:
//
// LOG_PIN, high signals that diagnostic logging messages
// should be sent to the USB port.
//
// FAST_PIN, high signals the emulation should run as fast
// as possible.  Low signals the emulation should run at a
// speed that approximates a 920M (typical instruction time
// of 11uS).
//
// These two pins are only read at the start of an emulation.
// If it is desired to change these parameters the Pico must
// be restarted either by removing, then replacing the USB power
// cable, or using the  button provided for this purpose.
//
// The onboard LED (LED_PIN) is used to signal emulation status.
// Immediately after loaded the LED is flashed 4 times to
// signal the emulation is ready to start.  When in the NOPOWER
// state the LED is extinguished.  When running the LED is lit
// continuously.

// If a catastrophic error occurs the LED shows
// a code indicating the error:
//    3 - attempt to address outside bounds of store
//    4 - attempt to execute an unimplemented type 14 instruction -
//    5 - attempt to execute an unimplemented type 15 instruction
//
// The Pico is equipped with a push button which can used used
// to force a restart of the Pico system.  This completely
// restarts the emulation system from the beginning.  It can be
// used to restart after a dynamic stop, infinite loop or
// catastrophic error.


/**********************************************************/
/*                     HEADER FILES                       */
/**********************************************************/


#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <pico/binary_info.h>
#include <tusb.h>
#include <setjmp.h>

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

#define RDR_1_PIN    0 // Set lsb of reader input
#define RDR_2_PIN    1 // these pins are assumed consecutive
#define RDR_4_PIN    2
#define RDR_8_PIN    3
#define RDR_16_PIN   4
#define RDR_32_PIN   5
#define RDR_64_PIN   6
#define RDR_128_PIN  7 // Set msb of reader input

#define RDR_PINS_MASK 0377 // PINs to bit mask 

#define PUN_1_PIN    8 // Set lsb of punch output
#define PUN_2_PIN    9 // these pins are assumed consecutive 
#define PUN_4_PIN   10
#define PUN_8_PIN   11
#define PUN_16_PIN  12
#define PUN_32_PIN  13
#define PUN_64_PIN  14 
#define PUN_128_PIN 15 // Pico sets to msb of punch output

#define PUN_PINS_MASK 0177400 // PINs to bit mask

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
#define EMULATION_ADDR_FAIL 3
#define EMULATION_14_FAIL   4
#define EMULATION_15_FAIL   5


/**********************************************************/
/*                         GLOBALS                        */
/**********************************************************/

UINT32 store[STORE_SIZE]; // core store

const UINT8 in_pins[IN_PINS] =  { NOPOWER_PIN,ACK_PIN, II_AUTO_PIN, LOG_PIN, FAST_PIN,
		                  RDR_1_PIN, RDR_2_PIN, RDR_4_PIN, RDR_8_PIN,
				  RDR_16_PIN, RDR_32_PIN, RDR_64_PIN, RDR_128_PIN };

const UINT8 out_pins[OUT_PINS] = { RDRREQ_PIN, PUNREQ_PIN, TTYSEL_PIN, PUN_1_PIN,
				   PUN_2_PIN, PUN_4_PIN, PUN_8_PIN, PUN_16_PIN,
				   PUN_32_PIN, PUN_64_PIN, PUN_128_PIN, LED_PIN };

UINT8 logging_enabled   = 1; // 1 = enable logging, 0 = disable logging to usb
UINT8 fast_enabled      = 0; // 1 = run at fastest speed, 0 = run at simulated
                             // 920M speed
UINT8 autostart_enabled = 0; // 0 = autostart after reset, 1 = run initial
                             // instructions after reset

static jmp_buf jbuf;  // used by setjmp in main

/**********************************************************/
/*                         FUNCTIONS                      */
/**********************************************************/


void                 emulate(const UINT16 start, UINT8 fast);
                                                   // run emulation from start
static inline void   check_address(const UINT16 add);
                                                   // check address is within bounds 
static inline void   load_initial_instructions();  // load initial orders
static inline UINT32 make_instruction(const UINT8 m, const UINT8 f, const UINT16 a);
                                                   //assemble an instruction
static inline void   set_up_gpios();               // initialize GPIO interface 
static inline void   led_on();                     // turn onboard LED on
static inline void   led_off();                    // turn onboard LED off
static void          emulation_fail(const char *msg, const UINT8 code);
                                                   // signal emulation hung on
                                                   // machine fault
static inline UINT8  logging();                    // TRUE is logging enabled
static inline UINT8  fast();                       // TRUE if running in fast mode
static inline UINT8  autostart();                  // TRUE if AUTOSTART, otherwise
                                                   // initial instructions
static inline UINT8  no_power();                   // TRUE if power off (NOPOWER TRUE)
static inline UINT8  ack();                        // status of ACK
static inline void   wait_for_power_on();          // wait for NOPOWER LOW
static inline void   wait_for_power_off();         // wait for NOPOWER HIGH
static inline void   wait_for_ack();               // wait for ACK HIGH
static inline void   wait_for_no_ack();            // wait for ACK LOW
static inline UINT8  get_pts_ch(UINT8 tty);        // read from paper tape station
static inline void   put_pts_ch(UINT8 ch, UINT8 tty);
                                                   // punch to paper tape station


void reader_test();
void punch_test();


/**********************************************************/
/*                           MAIN                         */
/**********************************************************/

int main() {

  int errors = 0;

  bi_decl(bi_program_description("Elliott 920M Emulator by Andrew Herbert"));

  stdio_init_all(); // initialise stdio
  set_up_gpios(); // configure interface to outside world
    
  while ( !tud_cdc_connected() ) sleep_ms(100); // wait for usb to wake up

  // long jump to here resets simulation
  setjmp(jbuf);

  // set local flags based on external inputs
  logging_enabled = logging();     // print logging messages to usb?
  fast_enabled    = fast();        // run at maximum speed?
  autostart_enabled = autostart(); // autostart or initial orders

  if ( logging_enabled )
    {
      puts("\n\n\nStarting");
      if ( fast_enabled )
	puts("Fast mode");
      else
	puts("Slow mode");
      if ( autostart_enabled )
	puts("Autostart");
      else
	puts("Initial instructions");
    }

  // 4 blinks to signal waking up
  for ( UINT8 i = 1 ; i <= 4 ; i++ )
    {
      led_on();
      sleep_ms(250);
      led_off();
      sleep_ms(250);
    }

  // wait for reset sequence from host
  wait_for_power_off();
  wait_for_power_on();

  // run tests
  reader_test();
  //punch_test();
    
  longjmp(jbuf, 0);
  
  while ( TRUE ) // run emulation for ever
  {
    if ( logging_enabled ) printf("\n\n\n\n\nPICO900 starting\n");
    wait_for_power_on(); 
    led_on(); // set LED to indicate running
    load_initial_instructions(); 
    emulate(8181, fast_enabled); // run emulation starting at location 8181
    // will only end up here on a failure or reset, in either case, restart
  }
}

/* The following three test routines are only for use during
the development phase of pico900.  They will not be used in the operational
system */

void punch_test()
{
  puts("Punch test starting");
  for ( UINT32 c = 0 ; c < 10000 ; c++ )
    {
      put_pts_ch(c%256, 0);
    }
  puts("Punch tests complete");
}


void reader_test()
{
  UINT8 errors = 0;
  puts("Reader test starting");
  // simple test loop emulating read
  for ( UINT32 c = 0 ; c < 1000 ; c++ )
    {
      //printf("Cycle %u\n", c);
      int ch = get_pts_ch(0);
      if  (ch != (c%256) ) 
	{
	  printf("Failed after %d got %d, expected %d\n",c, ch, c%256);
          c++;
	  if ( ++errors > 10 )
	    {
	      puts("Giving up");
	      longjmp(jbuf, 0);
	    }
	 }
      sleep_us(5);
    }
  puts("Reader test complete");
}


/**********************************************************/
/*                         EMULATION                      */
/**********************************************************/

/* 900 series emulator */

void emulate (const UINT32 start, UINT8 fast) {

  static UINT32 a_reg  = 0;
  static UINT32 q_reg  = 0;
  static UINT32 b_reg  = B_REG_LEVEL_1;
  static UINT32 sc_reg = SCR_LEVEL_1; // address in store of B static and SCR
  static UINT8  level = 1; // priority level
  static UINT32 instruction, f, a, m;
  static INT32  last_scr = -1; // scr of previous instruction

  if ( logging_enabled ) puts("Emulation starting\n");

  store[sc_reg] = start; // execution begins at location 'start'.

  // instruction fetch and decode loop
  while ( TRUE )
    {
      if ( no_power() )
	{
	  if ( logging_enabled ) puts("NOPOWER HIGH detected at instruction fetch");
	  longjmp(jbuf, 0); // reset signalled
	}

      if ( !fast ) sleep_us(11); // run at roughly 920M speed

      // increment SCR
      last_scr = store[sc_reg]++; // remember SCR

      // fetch and decode instruction;
      check_address(last_scr);
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
	    check_address(m);
	    q_reg = store[m]; store[b_reg] = q_reg;
	    break;

          case 1: // Add
	    check_address(m);
       	    a_reg = (a_reg + store[m]) & MASK_18;
	    break;

          case 2: // Negate and add
	    check_address(m);
	    a_reg = (UINT32) ((INT32) store[m] - (INT32) a_reg) & MASK_18;
	    break;

          case 3: // Store Q
	    check_address(m);
	    store[m] = q_reg >> 1;
	    break;

          case 4: // Load A
	    check_address(m);
	    a_reg = store[m];
	    break;

          case 5: // Store A
	    check_address(m);
	    if   ( level == 1 && m >= 8180 && m <= 8191 )
	      {
		break; // ignore write to II at level 1
	      }
	    else
	      store[m] = a_reg; 
	    break;

          case 6: // Collate
	    check_address(m);
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
	    check_address(m);
 	    store[m] = (store[m] + 1) & MASK_18;
	    break;

          case 11:  // Store S
	    {
	      check_address(m);
	      q_reg = store[sc_reg] & MOD_MASK;
	      store[m] = store[sc_reg] & ADDR_MASK;
	      break;
	    }

          case 12:  // Multiply
	    {
	      check_address(m);
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
	    }

          case 13:  // Divide
	    {
	      check_address(m);
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
	                UINT8 ch = get_pts_ch(1); 
	                a_reg = ((a_reg << 7) | ch) & MASK_18;
	                break;
	               }

	            case 2052: // read from teletype
		      {
	                UINT8 ch = get_pts_ch(1);
	                a_reg = ((a_reg << 7) | ch) & MASK_18;
	                break;
	              }

	            case 6144: // write to paper tape punch 
	              put_pts_ch((UINT8)(a_reg & 255), 0);
	              break;

	            case 6148: // write to teletype
	              put_pts_ch((UINT8)(a_reg & 255), 1);
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

/* check address within bounds */

static inline void check_address(const UINT16 m)
{
  if ( m >= STORE_SIZE)
    emulation_fail("**** Addressing beyond available store\n",
		   EMULATION_ADDR_FAIL);
}


/**********************************************************/
/*                    PAPER TAPE SYSTEM                   */
/**********************************************************/


/*  Input a character from paper tape station */

static inline UINT8 get_pts_ch(const UINT8 tty) {
  static UINT8 ch;
  //printf("Cycle %u no ack outstanding, issuing read request\n", cycle);
  if ( tty == 1 ) gpio_put(TTYSEL_PIN, 1);
  gpio_put(RDRREQ_PIN, 1);
  wait_for_ack();
  ch = (gpio_get_all() & RDR_PINS_MASK) >> RDR_1_PIN; // read 8 bits
  wait_for_no_ack();
  if ( tty ) gpio_put(TTYSEL_PIN, 0);
  gpio_put(RDRREQ_PIN, 0);
  sleep_us(2);
  return ch;
}

/* Output a character to the paper tape station */

static inline void put_pts_ch(const UINT8 ch, const UINT8 tty)
{
  printf("punching %d\n",
	 ((ch << PUN_1_PIN) & PUN_PINS_MASK) >> PUN_1_PIN);
  gpio_put_masked(PUN_PINS_MASK, ch << PUN_1_PIN); // write 8 bits
  if ( tty == 1 ) gpio_put(TTYSEL_PIN, 1);
  gpio_put(PUNREQ_PIN, 1); // request write
  wait_for_ack();
  wait_for_no_ack();
  if ( tty == 1 ) gpio_put(TTYSEL_PIN, 0);
  gpio_put(PUNREQ_PIN, 0);
  sleep_us(2);
}

/**********************************************************/
/*               INITIAL INSTRUCTIONS                     */
/**********************************************************/

/* Load initial instructions into sstore */

static inline void load_initial_instructions()
{
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

/* assemble an instruction */

static inline UINT32 make_instruction(const UINT8 m, const UINT8 f, const UINT16 a)
{
  return (((UINT32)m << 17) | ((UINT32)f << 13) | (UINT32)a);
}

/**********************************************************/
/*                         PICO GPIO                      */
/**********************************************************/


/*  initialize GPIOs - n.b. no point in pulling up/down inputs */

static inline void set_up_gpios()
{
  UINT32 in_pins_mask = 0, out_pins_mask = 0;
  // calculate masks
  for ( UINT8 i = 0; i < IN_PINS;  i++ ) in_pins_mask  |= (1 << in_pins[i]);
  for ( UINT8 i = 0; i < OUT_PINS; i++ ) out_pins_mask |= (1 << out_pins[i]);
  // initialize GPIOs
  gpio_init_mask(in_pins_mask | out_pins_mask);
  // set up GPIO directions
  gpio_set_dir_masked(in_pins_mask | out_pins_mask,
		      out_pins_mask); 
  // set all outputs 0
  gpio_clr_mask(out_pins_mask);
  // pull up NOPOWER
  gpio_pull_up(NOPOWER_PIN);
}

/* LED blinking */

static inline void led_on()
{
  gpio_put(LED_PIN, 1);
}

static inline void led_off()
{
  gpio_put(LED_PIN, 0);
}

// Fatal error
void emulation_fail(const char* msg, const UINT8 code) {
  // flash code then pause
  if ( logging_enabled )
    printf("Emulation failed: %s (%d)\n", msg);
  while ( TRUE )  // loop until reset flashing the code
    {
      sleep_ms(1000);
      for ( UINT8 j = 1 ; j <= code ; j++ )
        {
	  led_on();
           sleep_ms(250);
	 led_off();
	  sleep_ms(100);
        }
      if ( no_power() ) longjmp(jbuf,0); // restart on power cycle
    }
}

/* Read status pins */

static inline UINT8 autostart() {
  return gpio_get(II_AUTO_PIN);
}

static inline UINT8 fast() {
  return gpio_get(FAST_PIN);
}

static inline UINT8 logging() {
  return gpio_get(LOG_PIN);
}

static inline UINT8 no_power() {
  return gpio_get(NOPOWER_PIN);
}

static inline UINT8 ack() {
  return gpio_get(ACK_PIN);
}

/* Wait for power on signal (and absence of ack signal) from host */

static inline void wait_for_power_on() {
  while ( (no_power() == 1) ) sleep_ms(1);
  if ( logging_enabled ) puts("Power on detected");
}

/* Wait for host to remove power on and ack signal */

static inline void wait_for_power_off() {
  while ( (no_power() == 0) ) sleep_ms(1);
  if ( logging_enabled ) puts("Power off detected");
}


/* Wait for ACK to be 1 */

static inline void wait_for_ack() {
  while ( ack() == 0 ) {
    if ( no_power() )
      {
	if ( logging_enabled ) puts("NOPOWER while waiting for ACK HIGH");
	longjmp(jbuf, 0); // a reset has been signalled
      }
  }
}

/* wait for ACK to be 0 - not used */

static inline void wait_for_no_ack() {
  while ( ack() == 1 ) {
    if ( no_power() )
      {
	if ( logging_enabled ) puts("NOPOWER while waiting for ACK LOW");
	longjmp(jbuf, 0); // a reset has been signalled
      }
  }
}
