// Elliott 900 emulator for Raspberry Pi Pico
// Copyright (c) Andrew Herbert - 23/08/2023

// MIT Licence.

// Emulator for Elliott 920M computers, including secondary
// effects of 7 & 11 orders, and 920M shifts >= 48 places.

// Has simplified handling of priority levels and initial orders.
// Only supports paper tape input and output and teleprinter
// peripherals.

// This program is intended to run on a Raspberry Pi Pico and
// depends upon Pico specific functions and data types.

// The 900 store and registers are held in 32 bit variables.
// In the multiply (12) and divide (13) instructions calculations
// are performed using the A and Q registers as a 36 bit quantity
// and so the calculations for these two instructions use 64 bit
// quantities.  The results of each instruction, whether in a
// register or a store location are held masked to 18 bits. Thus
// the test for a negative number in a register or store location
// is to check if bit18 is set.  Addresses are generally hed as
// unsigned 16 bit quantities and characters as unsigned 8 bit
// quantities, but on the Pico both are implemented as 32 bit
// quantities so there is no saving of memory, just a reminder of
// the size of these types of data.

// The Pico emulates the paper tape station interface of a 920M 
// using GPIO pins.  The pin numbers are listed below.  The use 
// of each pin is as follows:
//
// RDRREQ_PIN, high signals a reader request.  The paper tape
// station is expected to load 8 bits of data on pins RDR_1_PIN
// (lsb) to  RDR_128_PIN (msb) and then raise ACK-PIN high for
// approximately 5uS to indicate the input data is ready. Once
// the computer has read in the data it lowers RDRReq-PIN to
// signal transfer complete.  
//
// PUNREQ_PIN, high signals a punch request. The paper tape
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
// the computer should wake up  in a fully reset state and
// execute an autostart or initial instructions as determined
// by II_AUTO.  NOPOWER should remain low so that the computer
// can continue to execute until next reset by raising NOPOWER
// high again.
//
// An additional GPIO pin, LOG_PIN, high signals that diagnostic
// logging messages should be sent to the serial port.
//
// These two pins are only read at the start of an emulation.
// If it is desired to change these parameters the Pico must
// be restarted either by removing, then replacing the USB power
// cable, or using the button provided for this purpose.
//
// The onboard LED (LED_PIN) is used to signal emulation status.
// A regular one second flash indicates emulation is running.
// A fast 0.25 second flash indicates halted after an internal
// error.


/**********************************************************/
/*                     HEADER FILES                       */
/**********************************************************/


#include <stdio.h>
#include<inttypes.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include <setjmp.h>

// Each 900 18 bit word is stored as a 32 bit unsigned value.
// For some operations we compute with a double word so we
// use 64 bit unsigned quantities for these. 


/**********************************************************/
/*                         DEFINES                        */
/**********************************************************/

// Booleans
#define TRUE  1
#define FALSE 0

#define TAPE  0  // select paper tape reader / punch
#define TTY   1  // select teletype

// Store size
#define STORE_SIZE 8192

// GPIO pins

// GPIO0, GPIO1 in uses as serial output
#define RDR_1_PIN    2 // Set lsb of reader input
#define RDR_2_PIN    3 // these pins are assumed consecutive
#define RDR_4_PIN    4
#define RDR_8_PIN    5
#define RDR_16_PIN   6
#define RDR_32_PIN   7
#define RDR_64_PIN   8
#define RDR_128_PIN  9 // Set msb of reader input

#define RDR_PINS_MASK (255 << RDR_1_PIN) // PINs to bit mask 

#define PUN_1_PIN   10 // Set lsb of punch output
#define PUN_2_PIN   11 // these pins are assumed consecutive 
#define PUN_4_PIN   12
#define PUN_8_PIN   13
#define PUN_16_PIN  14
#define PUN_32_PIN  15
#define PUN_64_PIN  16 
#define PUN_128_PIN 17 // Pico sets to msb of punch output

#define PUN_PINS_MASK (255 << PUN_1_PIN) // PINs to bit mask

#define NOPOWER_PIN 18 // set HIGH to stop and reset, set LOW to run
#define ACK_PIN     19 // pulse HIGH to acknowledge RDR or PUN request
#define II_AUTO_PIN 20 // set HIGH to autostart on reset, or LOW to execute
                       // initial instructions
#define TTYSEL_PIN  21 // Pico sets HIGH to request reader input and awaits ACK
#define PUNREQ_PIN  22 // Pico sets HIGH to request punch output and awiats ACK
// There is no GPIO23, GPIO24
// GPIO 25 is onboard LED    
#define RDRREQ_PIN  26 // Pico sets HIGH to select teleprinter,
                       // LOW for paper tape
#define LOG_PIN     27 // set HIGH to enable logging
#define LED_PIN     28 // external LED

#define RDRREQ_BIT   (1 << RDRREQ_PIN)
#define PUNREQ_BIT   (1 << PUNREQ_PIN)
#define TTYSEL_BIT   (1 << TTYSEL_PIN)
#define REQUEST_BITS (RDRREQ_BIT | PUNREQ_BIT | TTYSEL_BIT)
#define ACK_BIT      (1 << ACK_PIN)
#define NOPOWER_BIT  (1 << NOPOWER_PIN)
#define II_AUTO_BIT  (1 << II_AUTO_PIN)
#define LOG_BIT      (1 << LOG_PIN)
#define LED_BIT      (1 << LED_PIN)

// Useful constants
#define BIT_19    01000000
#define MASK_18   00777777
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
#define EMULATION_ADDR_FAIL    1 // invalid address detected
#define EMULATION_14_FAIL      2 // unimplmented 14 order case
#define EMULATION_15_FAIL      3 // unimplemented 15 order case
#define NOPOWER_FAIL           4 // NOPOWER detected


const char *error_messages[]  =
                          { "1 - addressing outside of store",
			    "2 - unknown 14 instruction",
			    "3 - unknown 15 instruction",
			    "4 - NOPOWER HIGH detected"};

#define INSTRUCTION_TIME 11 // approx 11uS per instruction, this can be
                            // changed to reflect other 900 machines, 
                            // but if reduced too far, emulation might
                            // overrun the paper tape station. 5uS is a
                            // practical minimum.
#define ACK_TIME          2 // shouldset to be identical to ACK_TIME in PTS

// The following are the periods for flashing the LED in various states

#define NO_BLINK       0
#define RAPID_BLINK  250
#define FAST_BLINK   500
#define SLOW_BLINK  1000


/**********************************************************/
/*                         GLOBALS                        */
/**********************************************************/

uint32_t store[STORE_SIZE]; // core store

#define IN_PINS  12 // count of input pins
#define OUT_PINS 12 // count of output pins

const uint32_t in_pins[IN_PINS] =
  { NOPOWER_PIN, ACK_PIN,    II_AUTO_PIN, LOG_PIN,
    RDR_1_PIN,   RDR_2_PIN,  RDR_4_PIN,   RDR_8_PIN,
    RDR_16_PIN,  RDR_32_PIN, RDR_64_PIN,  RDR_128_PIN };

const uint32_t out_pins[OUT_PINS] =
  { RDRREQ_PIN, PUNREQ_PIN, TTYSEL_PIN,  PUN_1_PIN,
    PUN_2_PIN,  PUN_4_PIN,  PUN_8_PIN,   PUN_16_PIN,
    PUN_32_PIN, PUN_64_PIN, PUN_128_PIN, LED_PIN };

uint32_t in_pins_mask  = 0; // initialized in setup_gpios()
uint32_t out_pins_mask = 0;

static jmp_buf jbuf;            // used by setjmp in main

static volatile uint32_t blink = NO_BLINK;

  
/**********************************************************/
/*                         FUNCTIONS                      */
/**********************************************************/

static void     e920m_emulation();             // emulator
static void     blinker();                     // status indicator
static void     jump_to(const uint32_t start); // start execution at start
                                               // address
static void     check_address(const uint32_t add);
                                               // check address is within
                                               // bounds 
static void     load_initial_instructions();   // load initial orders
static uint32_t make_instruction(const uint32_t m, const uint32_t f,
				 const uint32_t a);
                                               // assemble an instruction
static void     setup_gpios();                 // initialize GPIO interface 
static void     led_toggle();                  // toggle onboard LED
                                               // signal emulation hung on
                                               // machine fault
static uint32_t logging();                     // TRUE is logging enabled
static uint32_t autostart();                   // TRUE if AUTOSTART, otherwise
                                               // initial instructions
static uint32_t no_power();                    // TRUE if power off
                                               // (NOPOWER TRUE)
static void     wait_for_power_on();           // wait for NOPOWER LOW
static void     wait_for_power_off();          // wait for NOPOWER HIGH
static void     wait_for_ack();                // wait for ACK HIGH
static void     wait_for_no_ack();             // wait for ACK LOW
static uint32_t get_pts_ch(uint32_t tty);      // read from paper tape station
static void     put_pts_ch(uint32_t ch, uint32_t tty);
                                               // punch to paper tape station


/**********************************************************/
/*                           MAIN                         */
/**********************************************************/

int32_t main()
{
  bi_decl(bi_program_description("Elliott 920M Emulator by Andrew Herbert"));
  
  stdio_init_all(); // initialise stdio
  setup_gpios();    // configure interface to outside world
  sleep_us(250);
  printf("Hello!\n");
  multicore_launch_core1(blinker); // set LED blinker running
  e920m_emulation(); // run emulation
 }


/**********************************************************/
/*                         EMULATION                      */
/**********************************************************/

/* 900 series emulation */

static void e920m_emulation()
{
  int32_t fail_code;
  
  // long jump to here on error
  if ( fail_code = setjmp(jbuf) ) { // test if a longjmp occurred
    gpio_put_masked(REQUEST_BITS, 0); // clear any transfers

    if ( (fail_code == NOPOWER_FAIL) & logging() )
      printf("Pico900 - Restarting after NOPOWER\n");
      
    else if ( logging() ) {
       printf("Pico900 - Halted after error %s\n",
	      error_messages[fail_code-1]);
       blink = FAST_BLINK; // signal in error state
       wait_for_power_on(); // only restart after power cycle
    }
  }

  blink = NO_BLINK;

  if ( logging() ) {
    printf("Pico900 - Starting\n%s\n",
	   ( autostart() )
	   ? "Pico900 - Autostart selected"
	   : "Pico900 - Initial instructions selected");
  }

   wait_for_power_on(); // -NOPOWER enables execution

  if ( autostart() ) store[8177] = make_instruction(0,  8, 8177);

  if ( !autostart() ) load_initial_instructions();
  blink = SLOW_BLINK; // signal execution starting

  jump_to(autostart() ? 8177 : 8181); // start location depends on II_AUTO
  /* NOT REACHED */
}

/* execute program in store from start address */

static void jump_to (const uint32_t start) {

  uint32_t a_reg  = 0;
  uint32_t q_reg  = 0;
  uint32_t b_reg  = B_REG_LEVEL_1;
  uint32_t sc_reg = SCR_LEVEL_1; // address in store of B static and SCR
  uint32_t level = 1; // priority level
  uint32_t instruction, f, a, m;
  int32_t  last_scr = -1; // scr of previous instruction

  store[sc_reg] = start; // execution begins at location 'start'.

  // instruction fetch and decode loop
  while ( TRUE )
    {

      if ( no_power() ) {
	if (logging() )  printf("Pico900 - NOPOWER detected during "
				"program execution\n");
	longjmp(jbuf, NOPOWER_FAIL);
      }

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
	  // B-modification also affects Q but unsure how!
	}
      else
	  m = a & MASK_16;

      // perform function determined by function code f
      switch ( f )
        {

        case 0: // Load B
	    check_address(m);
	    q_reg = store[m]; store[b_reg] = q_reg;
	    sleep_us(INSTRUCTION_TIME);
	    break;

          case 1: // Add
	    check_address(m);
       	    a_reg = (a_reg + store[m]) & MASK_18;
	    sleep_us(INSTRUCTION_TIME);
	    break;

          case 2: // Negate and add
	    check_address(m);
	    q_reg = store[m];
	    a_reg =(q_reg - a_reg) & MASK_18;
	    sleep_us(INSTRUCTION_TIME);
	    break;

          case 3: // Store Q
	    check_address(m);
	    store[m] = q_reg >> 1;
	    sleep_us(INSTRUCTION_TIME);
	    break;

          case 4: // Load A
	    check_address(m);
	    a_reg = store[m];
	    sleep_us(INSTRUCTION_TIME);
	    break;

          case 5: // Store A
	    check_address(m);
	    if   ( level == 1  && m >= 8180 && m <= 8191)
	      {
		break; // ignore write to II at level 1
	      }
	    else
	      store[m] = a_reg; 
	    sleep_us(INSTRUCTION_TIME);
	    break;

          case 6: // Collate
	    check_address(m);
	    a_reg &= store[m];
	    sleep_us(INSTRUCTION_TIME);
	    break;

          case 7: // Jump if zero
  	    if   ( a_reg == 0 ) store[sc_reg] = m;
	    sleep_us(INSTRUCTION_TIME);
	    break;

          case 8: // Jump unconditional
	    store[sc_reg] = q_reg = m; // q_reg 920M secondary effect
	    sleep_us(INSTRUCTION_TIME);
	    break;

          case 9: // Jump if negative
	    if   ( a_reg >= BIT_18 ) store[sc_reg] = m;
	    sleep_us(INSTRUCTION_TIME);
	    break;

          case 10: // increment in store
	    check_address(m);
 	    store[m] = (store[m] + 1) & MASK_18;
	    sleep_us(INSTRUCTION_TIME);
	    break;

          case 11:  // Store S
	    {
	      check_address(m);
	      q_reg = store[sc_reg] & MOD_MASK;
	      store[m] = store[sc_reg] & ADDR_MASK;
	      sleep_us(INSTRUCTION_TIME);
	      break;
	    }

          case 12:  // Multiply
	    {
	      check_address(m);
	      // extend sign bits for a and store[m]
	      const int64_t al =
		(int64_t) ( ( a_reg >= BIT_18 )
		 	    ?  a_reg - BIT_19
		  	    :  a_reg );
	      const int64_t sl =
		(int64_t) ( ( store[m] >= BIT_18 )
			    ? store[m] - BIT_19
			    : store[m] );
	      int64_t prod = al * sl;
	      q_reg = (uint32_t) ((prod << 1) & MASK_18);
	      if   ( al < 0 ) q_reg |= 1;
	        prod = prod >> 17; // arithmetic shift
 	        a_reg = (uint32_t) (prod & MASK_18);
	      sleep_us(INSTRUCTION_TIME);
	      break;
	    }

          case 13:  // Divide
	    {
	      check_address(m);
	      // extend sign bit for aq
	      const int64_t al   =
		(int64_t) ( ( a_reg >= BIT_18 )
			    ?  a_reg - BIT_19
			    :  a_reg ); // sign extend
	      const int64_t ql   = (int64_t) q_reg;
	      const int64_t aql  = (al << 18) | ql;
	      const int64_t sl   =
		(int64_t) ( ( store[m] >= BIT_18 )
			    ? store[m] - BIT_19
			    :  store[m] );
              const int64_t quot = (( aql / sl) >> 1) & MASK_18;
	      const int32_t q = (int32_t) quot;
  	      a_reg = q | 1;
	      q_reg = q & 0777776;
	      sleep_us(INSTRUCTION_TIME);
	      break;
	    }

          case 14:  // Shift - assumes >> applied to a signed int64_t or int32_t
	            // is arithmetic -
	    {
              uint32_t places = m & ADDR_MASK;
	      uint32_t p = places & 63;
	      const uint64_t al  =
		(int64_t) ( ( a_reg >= BIT_18 )
			    ? a_reg - BIT_19
			    : a_reg ); // sign extend
	      const uint64_t ql  = q_reg;
	      uint64_t aql = (al << 18) | ql;
	      
	      if   ( places <= 2047 )
	        {
		  // 920M left shift
		  if   ( p > 48 ) p = p - 16;
	          aql <<= p;
	        }
	      else if ( places >= 6144 )
	        { // right shift is arithmetic
	          if ( (64 - p) <= 48 )
		    p = 64 - p;
		  else
		    p = 48 - p;
		  aql >>= p;
	        }  
	      else
	        {
	          longjmp(jbuf, EMULATION_14_FAIL);
	          /* NOT REACHED */
	        }

	      q_reg = (int32_t) (aql & MASK_18);
	      a_reg = (int32_t) ((aql >> 18) & MASK_18);
	      sleep_us(INSTRUCTION_TIME);
	      break;
	    }

            case 15:  // Input/output etc
	      {
                const uint32_t z = m & ADDR_MASK;
	        switch   ( z )
	    	  {

		    case 2048: // read from tape reader
		      { 
	                uint32_t ch = get_pts_ch(TAPE); 
	                a_reg = ((a_reg << 7) | ch) & MASK_18;
			sleep_us(INSTRUCTION_TIME - ACK_TIME);
	                break;
	               }

	            case 2052: // read from teletype
		      {
	                uint32_t ch = get_pts_ch(TTY);
	                a_reg = ((a_reg << 7) | ch) & MASK_18;
			sleep_us(INSTRUCTION_TIME - ACK_TIME);
	                break;
	              }

	            case 6144: // write to paper tape punch 
	              put_pts_ch((a_reg & 255), TAPE);
			sleep_us(INSTRUCTION_TIME - ACK_TIME);
	              break;

	            case 6148: // write to teletype
	              put_pts_ch((a_reg & 255), TTY);
			sleep_us(INSTRUCTION_TIME - ACK_TIME);
	              break;	      
	  
	            case 7168:  // Level terminate
	              level = 4;
	              sc_reg = SCR_LEVEL_4;
		      b_reg  = B_REG_LEVEL_4;
		      sleep_us(INSTRUCTION_TIME);
	              break;

	            default:
		      longjmp(jbuf, EMULATION_15_FAIL);
	              /* NOT REACHED */
		  } // end 15 switch
	      } // end case 15
	} // end function switch
    } // end while fetching and decoding instructions
}

/* check address within bounds */

static inline void check_address(const uint32_t m)
{
  if ( m >= STORE_SIZE) longjmp(jbuf, EMULATION_ADDR_FAIL);
}


/**********************************************************/
/*                    PAPER TAPE SYSTEM                   */
/**********************************************************/


/*  Input a character from paper tape station */

static  uint32_t get_pts_ch(const uint32_t tty) {
  uint64_t start = time_us_64();
  uint32_t ch, request = RDRREQ_BIT | ( tty ? TTYSEL_BIT : 0 );
  blink = RAPID_BLINK;
  gpio_put_masked(REQUEST_BITS, request);
  wait_for_ack();
  ch = (gpio_get_all() >> RDR_1_PIN) & 255; // read 8 bits
  gpio_put_masked(REQUEST_BITS, 0);
  wait_for_no_ack();
  blink = SLOW_BLINK;
  // no wait here - assume INSTRUCTION_TIME > duration of ACK
  if ( logging() )
    printf("R%3d\n", ch);
  return ch;
}

/* Output a character to the paper tape station */

static void put_pts_ch(const uint32_t ch, const uint32_t tty)
{
  uint64_t start = time_us_64();
  uint32_t request =  PUNREQ_BIT | ( tty ? TTYSEL_BIT : 0 )
                           | (ch << PUN_1_PIN);
  if ( logging() )
    printf("P %3d\n", ch);
  blink = RAPID_BLINK;
  gpio_put_masked(REQUEST_BITS | PUN_PINS_MASK, request);
  wait_for_ack();
  gpio_put_masked(REQUEST_BITS, 0);
  wait_for_no_ack();
  blink = SLOW_BLINK;
  // no wait here - assume INSTRUCTION_TIME > duration of ACK
}


/**********************************************************/
/*               INITIAL INSTRUCTIONS                     */
/**********************************************************/


/* Load initial instructions into store */

static void load_initial_instructions()
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
}

/* assemble an instruction */

static inline uint32_t make_instruction(const uint32_t m, const uint32_t f,
					const uint32_t a)
{
  return (((uint32_t)m << 17) | ((uint32_t)f << 13) | (uint32_t)a);
}


/**********************************************************/
/*                         PICO GPIO                      */
/**********************************************************/


/*  initialize GPIOs */

static void setup_gpios()
{
  // calculate masks
  for ( uint32_t i = 0; i < IN_PINS;  i++ ) in_pins_mask  |= (1 << in_pins[i]);
  for ( uint32_t i = 0; i < OUT_PINS; i++ ) out_pins_mask |= (1 << out_pins[i]);
  // initialize GPIOs
  gpio_init_mask(in_pins_mask | out_pins_mask);
  // set up GPIO directions
  gpio_set_dir_masked(in_pins_mask | out_pins_mask,
  		      out_pins_mask);
  // set all outputs to LOW
  gpio_put_masked(out_pins_mask, 0);
  // set pull up on NOPOWER to avoid spurious indications
  gpio_pull_up(NOPOWER_PIN);
  // set pull up on LOG_PIN to default to logging on
  gpio_pull_up(LOG_PIN);
  // set pull down on ACK amd II_AUTO to avoid spurious indications
  gpio_pull_down(ACK_PIN);
  gpio_pull_down(II_AUTO_PIN);
}

static inline uint32_t autostart()
{
  return gpio_get(II_AUTO_PIN);
}

static inline uint32_t logging()
{
  /* return gpio_get(LOG_PIN); */
  return gpio_get(LOG_PIN);
}

static inline uint32_t no_power() // TRUE if power not connected
{
  return gpio_get(NOPOWER_PIN);
}

/* Wait for power on signal (and absence of ack signal) from host */

static inline void wait_for_power_on()
{
  if ( logging() ) printf("Pico900 - Waiting for NOPOWER LOW\n");
  while ( TRUE ) {
    while ( no_power() ) sleep_ms(100); // wait for NOPOWER FALSE
    sleep_ms(100); // debounce switch
    if ( no_power() ) {
      continue; // not stable yet
    }
    else {
      break; // stable
	}
  }  
  if ( logging() ) printf("Pico900 - NOPOWER LOW detected\n");
}

static inline void wait_for_power_off()
{
  if ( logging() ) printf("Pico900 - Waiting for NOPOWER HIGH\n");
  while ( TRUE ) { 
    while ( !no_power() ) sleep_ms(100); // wait for NOPOWER TRUE
    sleep_ms(100); // debounce switch
    if ( !no_power() ) {
      continue; // not stable yet
    }
    else {
      break; // stable
	}
  }  
  if ( logging() ) printf("Pico900 - NOPOWER HIGH detected\n");
}

/* Wait for ACK to be 1  */

static inline void wait_for_ack()
{
  while ( TRUE ) { // wait for ACK high
    if ( gpio_get(ACK_PIN) ) return;
    if ( no_power() ) {
      if ( logging ) printf("Pico900 - NOPOWER while waiting for ACK\n");
      longjmp(jbuf, NOPOWER_FAIL);
    }
  }
}

static inline void wait_for_no_ack()
{
  while ( TRUE ) { // wait for ACK high
    if ( !gpio_get(ACK_PIN) ) return;
    if ( no_power() ) {
      if ( logging ) printf("Pico900 - NOPOWER while waiting for no ACK\n");
      longjmp(jbuf, NOPOWER_FAIL);
    }
  }
}

/**********************************************************/
/*                          BLINKER                       */
/**********************************************************/

static inline void blinker()
{
  static uint32_t led_state = 0;
  while ( TRUE ){
    if ( blink == NO_BLINK ) {
      led_state = 0;
      gpio_put(LED_PIN, 0);
      sleep_ms(RAPID_BLINK);
    }
    else {	
      led_state += 1;
      gpio_put(LED_PIN, led_state%2);
      sleep_ms(blink);
    }
  }
}
      
