/*  Filename:       ShutterBits.c
    Author:         Corey Davyduke
    Created:        2012-04-30
    Modified:       2012-04-30
    Description:    This project is based upon the "timer" project found
    under the Gel examples.  The HC11 uses the serial port to prompt the
    user for the current time and uses the timer to keep time thereafter.
    This project is an excellent use of interrupts.  I have modified the
    original project to blink an LED every second as well as to print
    the time out to an LCD display.
*/

#include "ShutterBits.h"
//#include "LCD.h"

#ifdef USE_INTERRUPT_TABLE

/* Interrupt table used to connect our timer_interrupt handler.

   Note: the `XXX_handler: foo' notation is a GNU extension which is
   used here to ensure correct association of the handler in the struct.
   This is why the order of handlers declared below does not follow
   the HC11 order.  */
struct interrupt_vectors __attribute__((section(".vectors"))) vectors = 
{
  res0_handler:           fatal_interrupt, /* res0 */
  res1_handler:           fatal_interrupt,
  res2_handler:           fatal_interrupt,
  res3_handler:           fatal_interrupt,
  res4_handler:           fatal_interrupt,
  res5_handler:           fatal_interrupt,
  res6_handler:           fatal_interrupt,
  res7_handler:           fatal_interrupt,
  res8_handler:           fatal_interrupt,
  res9_handler:           fatal_interrupt,
  res10_handler:          fatal_interrupt, /* res 10 */
  sci_handler:            fatal_interrupt, /* sci */
  spi_handler:            fatal_interrupt, /* spi */
  acc_overflow_handler:   fatal_interrupt, /* acc overflow */
  acc_input_handler:      fatal_interrupt,
  timer_overflow_handler: fatal_interrupt,
  output5_handler:        fatal_interrupt, /* out compare 5 */
  output4_handler:        fatal_interrupt, /* out compare 4 */
  output3_handler:        fatal_interrupt, /* out compare 3 */
  output2_handler:        fatal_interrupt, /* out compare 2 */
  output1_handler:        fatal_interrupt, /* out compare 1 */
  capture3_handler:       fatal_interrupt, /* in capt 3 */
  capture2_handler:       fatal_interrupt, /* in capt 2 */
  capture1_handler:       fatal_interrupt, /* in capt 1 */
  irq_handler:            fatal_interrupt, /* IRQ */
  xirq_handler:           fatal_interrupt, /* XIRQ */
  swi_handler:            fatal_interrupt, /* swi */
  illegal_handler:        fatal_interrupt, /* illegal */
  cop_fail_handler:       fatal_interrupt,
  cop_clock_handler:      fatal_interrupt,

  /* What we really need.  */
  rtii_handler:           timer_interrupt,
  reset_handler:          _start
};

#endif

#define TIMER_DIV  (8192L)
#define TIMER_TICK (M6811_CPU_E_CLOCK / TIMER_DIV)

// BIT 4 and 5 on PORTA are for the H-bridge driver of the shutter.
#define PA4 (1<<4)
#define PA5 (1<<5)

#define LINE_1      0x80                // beginning position of LCD line 1
#define LINE_2      0xC0                // beginning position of LCD line 2
#define LINE_3      0x94                // beginning position of LCD line 3
#define LINE_4      0xD4                // beginning position of LCD line 4

// I/O Port Addresses
#define LCD_CMD  *(unsigned char *)(0xB5F0)
#define LCD_DAT  *(unsigned char *)(0xB5F1)

unsigned long timer_count;
unsigned long boot_time;

unsigned short shutter_open;

void LCD_Command(unsigned char cval);
void LCD_busy(void);
void cprint(char dval);
void LCDprint(char *sptr);
void LCD_Initialize(void);

/* Timer interrupt handler.  */
void __attribute__((interrupt)) timer_interrupt (void)
{
  timer_count++;
  timer_acknowledge ();
}

/* Returns the current number of ticks that ellapsed since we started.  */
static inline unsigned long timer_get_ticks ()
{
  unsigned long t;

  lock ();
  t = timer_count;
  unlock ();
  return t;
}

/* Translate the number of ticks into some seconds.  */
static unsigned long timer_seconds (unsigned long ntime)
{
  unsigned long n;

  /* To compute SECS = NTIME * TIMER_DIV / M6811_CPU_E_CLOCK accurately,
     use Bezous relation (A = BQ + R).  */
  n = ntime * (TIMER_DIV / M6811_CPU_E_CLOCK);
  n += (ntime * (TIMER_DIV % M6811_CPU_E_CLOCK)) / M6811_CPU_E_CLOCK;
  n += boot_time;
  return n;
}

/* Translate the number of ticks into some microseconds.  */
static unsigned long timer_microseconds (unsigned long ntime)
{
  unsigned long n;

  /* To compute SECS = NTIME * TIMER_DIV / M6811_CPU_E_CLOCK accurately,
     use Bezous relation (A = BQ + R).  */
  n = ntime * (TIMER_DIV / 2);
  n += (ntime * (TIMER_DIV % 2)) / 2;
  n = n % 1000000L;
  return n;
}

/* Translate the string pointed to by *p into a number.
   Update *p to point to the end of that number.  */
static unsigned short get_value (char **p)
{
  char *q;
  unsigned short val;
  
  q = *p;
  while (*q == ' ')
    q++;
  
  val = 0;
  while (1)
  {
    char c = *q++;
    if (c < '0' || c > '9')
      break;
    val = (val * 10) + (c - '0');
  }
  q--;
  *p = q;
  return val;
}

/* Ask for the boot time.  */
static void get_time ()
{
  char buf[32];
  int pos;
  char c;
  unsigned short hours, mins, secs;
  char *p;
  int error = 0;
  
  print ("\r\nBoot time ? ");
  pos = 0;
  while (1)
  {
    c = serial_recv ();
    if (c == '\r' || c == '\n')
      break;

    if (c == '\b')
    {
      print ("\b \b");
      pos--;
      if (pos < 0)
        pos = 0;
    }
    else if (pos < sizeof (buf) - 1)
    {
      buf[pos] = c;
      buf[pos+1] = 0;
      print (&buf[pos]);
      pos++;
    }
  }

  print ("\n");
  buf[pos] = 0;
  p = buf;
  hours = get_value (&p);
  if (*p++ != ':')
    error = 1;
  mins = get_value (&p);
  if (*p++ != ':' || mins >= 60)
    error = 1;
  secs = get_value (&p);
  if (*p++ != 0 || secs >= 60)
    error = 1;

  if (error == 0)
  {
    boot_time = (hours * 3600) + (mins * 60) + (secs);
    print ("Boot time is set.\r\n");
  }
  else
  {
    print ("Invalid boot time.\r\n");
    print ("Format is: HH:MM:SS\r\n");
  } 
}

/* Display the current time on the serial line.  */
static void display_time (unsigned long ntime)
{
  unsigned long seconds;
  unsigned short hours, mins;
  unsigned long nus;
  char buf[12];
  char buf2[32];
  char buf3[32];

  static unsigned long last_sec = 0xffffffff;
  static unsigned long last_us = 0;

  /* Translate the number of ticks in seconds and milliseconds.  */
  seconds = timer_seconds (ntime);
  nus = timer_microseconds (ntime);
          
  sprintf (buf3, "s=%ld, nus=%ld", seconds, nus);

  nus = nus / 100000L;

  /* If the seconds changed, re-display everything.  */
  if (seconds != last_sec)
  {
    last_sec = seconds;
    last_us = nus;
    hours = (unsigned short) (seconds / 3600L);
    mins = (unsigned short) (seconds % 3600L);
    seconds = (unsigned long) (mins % 60);
    mins = mins / 60;
    buf[0] = '0' + (hours / 10);
    buf[1] = '0' + (hours % 10);
    buf[2] = ':';
    buf[3] = '0' + (mins / 10);
    buf[4] = '0' + (mins % 10);
    buf[5] = ':';
    buf[6] = '0' + (seconds / 10);
    buf[7] = '0' + (seconds % 10);
    buf[8] = '.';
    buf[9] = '0' + nus;
    buf[10] = 0;
    serial_print ("\r");
    serial_print (buf);
	  
	  if (shutter_open)
	  {
      // Open the shutter.
      _io_ports[M6811_PORTA] |= PA5;
      _io_ports[M6811_PORTA] &= ~PA4;
      shutter_open = 0;
	  }
	  else
	  {
      // Close the shutter.
      _io_ports[M6811_PORTA] |= PA4;
      _io_ports[M6811_PORTA] &= ~PA5;
	    shutter_open = 1;
	  }

    // Write the time out to the LCD display.
    LCD_Command(LINE_2);               // goto lcd line 2
    LCDprint(buf);

    // Write the timer count out to the LCD display for diagnostic purposes.
    sprintf (buf2, "%ld", timer_count);
    LCD_Command(LINE_3);               // goto lcd line 3
    LCDprint(buf2);

    // Write the number of microseconds out to the LCD display for diagnostic purposes.
    LCD_Command(LINE_4);               // goto lcd line 4
    LCDprint(buf3);
  }

  /* Only re-display the tens of a second.  */
  else if (last_us != nus)
  {
    last_us = nus;
    buf[0] = '0' + nus;
    buf[1] = 0;
    serial_print ("\b");
    serial_print (buf);
  }
  serial_flush ();
}

// Wait for the LCD busy pin to clear
void LCD_busy()
{
  while ((LCD_CMD & 0x80)) ;
}

void LCD_Command(unsigned char cval)
{
  LCD_busy();                         // wait for busy to clear
  LCD_CMD = cval;                     // ouptut command
}

void LCD_Initialize(void)
{
  // Initialize the LCD
  LCD_Command(0x3C);                 // initialize command
  LCD_Command(0x0C);                 // display on, cursor off
  LCD_Command(0x06);
  LCD_Command(0x01);
}

//* LCD Display Character
void cprint(char dval)
{
  LCD_busy();                         // wait for busy to clear
  LCD_DAT = dval;                     // ouptut data
}

// LCD Display String
void LCDprint(char *sptr)
{
	while( *sptr )
  {
		cprint(*sptr);
		++sptr;
	}
}

int main ()
{
  unsigned long prev_time;
  
  serial_init ();
  lock ();
  boot_time = 0;
  timer_count = 0;

  /* Set interrupt handler for bootstrap mode.  */
  set_interrupt_handler (RTI_VECTOR, timer_interrupt);

  /* Initialize the timer.  */
  timer_initialize_rate (M6811_TPR_16);
  prev_time = timer_count;

  unlock ();

  // Get the LCD ready for use.
  LCD_Initialize();

  // Print the "welcome" message out the serial port and on the LCD.
  print ("\nHello, world!\n");
  LCD_Command(LINE_1);               // goto lcd line 1
  LCDprint("Hello, world!");

  /* Ask for the boot time.  */
  get_time ();
  
  shutter_open = 1;

  /* Loop waiting for the time to change and redisplay it.  */
  while (1)
  {
    unsigned long ntime;

    /* Reset the COP (in case it is active).  */
    cop_optional_reset ();

    /* If something is received on the serial line,
       ask for the boot time again.  */
    if (serial_receive_pending ())
      get_time ();

    /* Get current time and see if we must re-display it.  */
    ntime = timer_get_ticks ();
    if (ntime != prev_time)
    {
      prev_time = ntime;
      display_time (ntime);
    }
  }
}
