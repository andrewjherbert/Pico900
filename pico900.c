// Elliott 900 emulator for Raspberry Pi Pico - Andrew Herbert - 03/04/2021

// Emulator for Elliott 900 Series computers.
// Does not implement 'undefined' effects.
// Has simplified handling of priority levels and initial orders.


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
#define STORE_SIZE 8192u

// GPIO pins
#define NOPOWER_PIN  1
#define ACK_PIN      2
#define RDREQ_PIN    4
#define WRREQ_PIN    5
#define TTYSEL_PIN   6
#define RDR_1_PIN    9
#define RDR_2_PIN   10
#define RDR_4_PIN   11
#define RDR_8_PIN   12
#define RDR_16_PIN  14
#define RDR_32_PIN  15
#define RDR_64_PIN  16
#define RDR_128_PIN 17
#define PUN_1_PIN   35
#define PUN_2_PIN   34
#define PUN_4_PIN   32
#define PUN_8_PIN   31
#define PUN_16_PIN  29
#define PUN_32_PIN  27
#define PUN_64_PIN  26
#define LED_PIN     25 // on board LED
#define PUN_128_PIN
#define II_AUTO_PIN 22



// Useful constants
#define BIT_19    01000000u
#define MASK_18   0777777u
#define MASK_18L  0777777uL
#define BIT_18    00400000u
#define MASK_16   00177777u
#define ADDR_MASK 8191u
#define MOD_MASK  00160000u
#define MOD_SHIFT 13u
#define FN_MASK   15u
#define FN_SHIFT  13u

// Locations of B register and SCR for priority levels 1 and 4
#define SCR_LEVEL_1   0u
#define SCR_LEVEL_4   6u
#define B_REG_LEVEL_1 1u
#define B_REG_LEVEL_4 7u

// Codes for emulation failures
#define EMULATION_SCR_FAIL 3u
#define EMULATION_14_FAIL  4u
#define EMULATION_15_FAIL  5u


/**********************************************************/
/*                         GLOBALS                        */
/**********************************************************/

UINT32 store[STORE_SIZE]; 


/**********************************************************/
/*                         FUNCTIONS                      */
/**********************************************************/


void   wait_for_power();             // Wait for NOPOWER = 0
void   wait_for_no_power();          // Wait for NOPOWER = 1
void   emulate(UINT32 start);        // run emulation
void   clear_store();                // clear main store
UINT8  read_tape();                  // read from paper tape
void   punch_tape(UINT8 ch);         // punch to paper tape
UINT8  read_tty();                   // read from teletype
void   write_tty(UINT8 ch);          // write to teletype
void   load_initial_instructions();    // load initial orders
INT32  make_instruction(UINT16 m, UINT16 f, UINT16 a);
                                     // helper for load_initial_instructions
void   set_up_gpios();               // initial GPIO interface to outside world
void   emulation_fail(UINT8 code);   // e.g., illegal instruction encountered
void   slow_blink();                 // signal waiting for NOPOWER = 0
void   steady_glow();                // signal running
UINT32 no_power();                   // status of NOPOWER


/**********************************************************/
/*                           MAIN                         */
/**********************************************************/


INT32 main () {

  bi_decl(bi_program_description("This is a test binary"));
  bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));

  stdio_init_all();

  set_up_gpios(); // configure interface to outside world
  while (TRUE)
    {
      puts("PICO900 starting");
      wait_for_power(); // wait for external world to reset us
      clear_store();
      load_initial_instructions(); // load initial instructions into store
      emulate(8181);  // run emulation starting at location 8181
    }
}


/**********************************************************/
/*                         EMULATION                      */
/**********************************************************/


void emulate (UINT32 start) {

  static UINT32 a_reg  = 0;
  static UINT32 q_reg  = 0;
  static UINT32 b_reg  = B_REG_LEVEL_1;
  static UINT32 sc_reg = SCR_LEVEL_1; // address in store of B static and SCR
  static UINT8  level = 1; // priority level
  static UINT32 instruction, f, a, m;
  static INT32  last_scr = -1; // scr of previous instruction
  static UINT32  s; // current SCR value
  static UINT64 al, ql, aql, sl, prod, quot; // workspace
  static UINT32 z;
  static UINT32 places;
  static UINT32 ws;
  static UINT8 ch;

  store[sc_reg] = start; // execution begins at location 'start'.

  // instruction fetch and decode loop
  while ( TRUE )
    {
      if ( no_power() ) return;
      
      // increment SCR
      last_scr = store[sc_reg];         // remember SCR
      if   ( last_scr >= STORE_SIZE )
        {
	  printf("*** SCR has overflowed the store (SCR = %d)\n", last_scr);
          emulation_fail(EMULATION_SCR_FAIL);
 	  /* NOT REACHED */
        }
      s = ++store[sc_reg];                 // increment SCR

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
  	    if   ( a_reg == 0 ) store[sc_reg] = s = m;
	    break;

          case 8: // Jump unconditional
	    store[sc_reg] = s = m;
	    break;

          case 9: // Jump if negative
	    if   ( a_reg >= BIT_18 ) store[sc_reg] = s = m;
	    break;

          case 10: // increment in store
 	    store[m] = (store[m] + 1) & MASK_18;
	    break;

          case 11:  // Store S
	    {
              ws = store[sc_reg];
	      q_reg = ws & MOD_MASK;
	      store[m] = ws & ADDR_MASK;
	      break;
	    }

          case 12:  // Multiply
	    {
	      // extend sign bits for a and store[m]
	      al = ( ( a_reg >= BIT_18 )
			   ? ((INT64) a_reg - (INT64) BIT_19)
			   : (INT64) a_reg );
	      sl = ( ( store[m] >= BIT_18 )
			   ? store[m] - BIT_19
				       : store[m] );
	      prod = al * sl;
	      q_reg = (UINT32) ((prod << 1) & MASK_18L );
	      if   ( al < 0 ) q_reg |= 1;
	      prod = prod >> 17; // arithmetic shift
 	      a_reg = (UINT32) (prod & MASK_18L);
	      break;
	    }

          case 13:  // Divide
	    {
	      // extend sign bit for aq
	      al   = ( ( a_reg >= BIT_18 )
			    ? (INT64) a_reg - (INT64) BIT_19
			    : (INT64) a_reg ); // sign extend
	      ql   = (INT64) q_reg;
	      aql  = (al << 18) | ql;
	      sl   = ( ( store[m] >= BIT_18 )
			    ? (INT64) store[m] - (INT64) BIT_19
			    : (INT64) store[m] );
              quot = (( aql / sl) >> 1) & MASK_18L;
	      ws   = (UINT32) quot;
  	      a_reg = ws | 1;
	      q_reg = ws & 0777776;
	      break;
	    }

          case 14:  // Shift - assumes >> applied to a signed INT64 or INT32
	            // is arithmetic
	    {
              places = m & ADDR_MASK;
	      al  = ( ( a_reg >= BIT_18 )
			     ? (INT64) a_reg - (INT64) BIT_19
			     : (INT64) a_reg ); // sign extend
	      ql  = q_reg;
	      aql = (al << 18) | ql;
	      
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
	          printf("*** Unsupported i/o 14 i/o instruction\n");
	          emulation_fail(EMULATION_14_FAIL);
	          /* NOT REACHED */
	        }

	      q_reg = (INT32) (aql & MASK_18L);
	      a_reg = (INT32) ((aql >> 18) & MASK_18L);
	      break;
	    }

            case 15:  // Input/output etc
	      {
                z = m & ADDR_MASK;
	        switch   ( z )
	    	  {

		    case 2048: // read from tape reader
		      { 
	                ch = read_tape(); 
	                a_reg = ((a_reg << 7) | ch) & MASK_18;
	                break;
	               }

	            case 2052: // read from teletype
		      {
	                ch = read_tty();
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
	              printf("*** Unsupported 15 i/o instruction\n");
		      emulation_fail(EMULATION_15_FAIL);
	              /* NOT REACHED */
		  } // end 15 switch
	      } // end case 15
	} // end function switch
    } // end while fetching and decoding instructions
}

void clear_store() {
  static UINT32 i;
  for ( i=0 ; i < STORE_SIZE ; ++i ) store[i] = 0;
}

/**********************************************************/
/*                    PAPER TAPE SYSTEM                   */
/**********************************************************/


/* Paper tape reader */
UINT8 read_tape() {
  puts("Read from paper tape\n");
  return 0;   // Too keep gcc happy
}

/* paper tape punch */
void punch_tape(UINT8 ch) {
  puts("Punch to paper tape\n");
  return;
}

/* Teletype */
UINT8 read_tty() {
    return 0;   // Too keep gcc happy
}

void write_tty(UINT8 ch) {
  puts("Type to teleprinter\n");
  return;
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

INT32 make_instruction(UINT32 m, UINT32 f, UINT32 a) {
  return ((m << 17) | (f << 13) | a);
}

/**********************************************************/
/*                         PICO                           */
/**********************************************************/


void set_up_gpios() {
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
}

void emulation_fail(UINT8 code) {
  // flash code then pause
  while (TRUE) {
    printf("Emulation failed: %d\n", code);
  }
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

UINT32 no_power() {
  return FALSE;
}

void wait_for_power() {
  UINT64 count = 0;
  while ( TRUE ) {
    if ( (++count % 5) == 0 ) puts("Waiting for NOPOWER to go to 0\n");
    slow_blink();
  }
}

void wait_for_no_power() {
  return;
}
