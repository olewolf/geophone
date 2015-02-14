/****************************************************************************
 * geosampler.ino
 *
 * Record geophone data and output the data (from 0 through 4095) to the
 * serial port.
 *
 * See the file COPYRIGHT for copyright details.
 ***************************************************************************/


/* Serial speed for the report generation.  It should be fast enough to
   allow several values to be passed per second.  A speed of 38,400 baud
   should suffice for worst case reports of about 2,600 bytes per second. */
#define SERIAL_SPEED    115200

/* The geophone data is sampled on analog pin 5. */
#define GEODATA_PIN          5

/* Make an LED blink on every successful report. */
#define REPORT_BLINK_ENABLED   1
#define REPORT_BLINK_LED_PIN  13

/* Define the geophone data sampling rate. */
#define SAMPLE_RATE   512

/* Define the on-board LED so we can turn it off. */
#define LED_PIN             13

/* Create a double buffer for geodata samples. */
#define NUMBER_OF_GEODATA_SAMPLES 256
short geodata_samples[ NUMBER_OF_GEODATA_SAMPLES * 2 ];
short *geodata_samples_real;
/* Indexes used by the interrupt service routine. */
int  isr_current_geodata_index;
/* Semaphor indicating that a frame of geophone samples is ready. */
bool geodata_buffer_full;
/* Flag that indicates that a report with amplitude information was
   created.  It is used by the report LED blinking. */
bool report_was_created;


/**
 * Setup the timer interrupt and prepare the geodata sample buffers for
 * periodic sampling.  Timer1 is used to generate interrupts at a rate of
 * 512 Hz.
 *
 * This function is board specific; if other board than the Arduino Mega
 * or the Arduino Due are used the code must be updated.
 */
void start_sampling( )
{
  /* Prepare the buffer for sampling. */
  isr_current_geodata_index = 0;
  geodata_buffer_full       = false;

  /* Setup interrupts for the Arduino Mega. */
#if defined( ARDUINO_AVR_MEGA2560 ) || defined( ARDUINO_AVR_UNO ) || defined( ARDUINO_AVR_DUEMILANOVE )
  // Set timer1 interrupt to sample at 512 Hz. */
  const unsigned short prescaling     = 1;
  const unsigned short match_register = F_CPU / ( prescaling * SAMPLE_RATE ) - 1;
  cli( );
  TCCR1B = ( TCCR1B & ~_BV(WGM13) ) | _BV(WGM12);
  TCCR1A = TCCR1A & ~( _BV(WGM11) | _BV(WGM10) );
  TCCR1B = ( TCCR1B & ~( _BV(CS12) | _BV(CS11) ) ) | _BV(CS10);
  OCR1A = match_register;
  TIMSK1 |= _BV(OCIE1A);
  sei( );

  /* Setup interrupts the Arduino Due. */
#elif defined( ARDUINO_SAM_DUE )
  /* Set a 12-bit resolutiong. */
  analogReadResolution( 12 );
  /* Disable write protect of PMC registers. */
  pmc_set_writeprotect( false );
  /* Enable the peripheral clock. */
  pmc_enable_periph_clk( TC3_IRQn );
  /* Configure the channel. */
  TC_Configure( TC1, 0, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4 );
  uint32_t rc = VARIANT_MCK/128/SAMPLE_RATE;
  /* Setup the timer. */
  TC_SetRA( TC1, 0, rc/2 );
  TC_SetRC( TC1, 0, rc );
  TC_Start( TC1, 0 );
  TC1->TC_CHANNEL[ 0 ].TC_IER = TC_IER_CPCS;
  TC1->TC_CHANNEL[ 0 ].TC_IDR = ~TC_IER_CPCS;
  NVIC_EnableIRQ( TC3_IRQn );
#else
#error Arduino board not supported by this software.
#endif
}



#if defined( ARDUINO_AVR_MEGA2560 ) || defined( ARDUINO_AVR_UNO ) || defined( ARDUINO_AVR_DUEMILANOVE )
/**
 * Interrupt service routine for Arduino Mega devices which invokes the
 * generic interrupt service routine.
 */
ISR(TIMER1_COMPA_vect)
{
  sampling_interrupt( );
}


#elif defined( ARDUINO_SAM_DUE )
/**
 * Interrupt service routine for Arduino Due devices which invokes the
 * generic interrupt service routine.
 */
void TC3_Handler( )
{
  TC_GetStatus( TC1, 0 );
  sampling_interrupt( );
}


#else
#error Arduino board not supported by this software.
#endif



/*
 * Interrupt service routine for sampling the geodata.  The geodata analog
 * pin is sampled at each invokation of the ISR.  If the buffer is full, a
 * pointer is passed to the main program and a semaphor is raised to indicate
 * that a new frame of samples is available, and future samples are written
 * to the other buffer.
 *
 * While not a sampling task, we take advantage of the timer interrupt to
 * blink the report LED if enabled.
 */
void sampling_interrupt( )
{
  /* Read a sample and store it in the geodata buffer. */
#if defined( ARDUINO_AVR_MEGA2560 ) || defined( ARDUINO_AVR_UNO ) || defined( ARDUINO_AVR_DUEMILANOVE )
  const int adc_resolution = 1024;
#elif defined( ARDUINO_SAM_DUE )
  const int adc_resolution = 4096;
#endif
  short geodata_sample = analogRead( GEODATA_PIN ) - ( adc_resolution >> 1 );
  /* Scale the sample. */
  const int scale = 8192 / adc_resolution;
  geodata_sample = (short)( (double)geodata_sample * scale );
  geodata_samples[ isr_current_geodata_index++ ] = geodata_sample;

  /* Raise a semaphor if the buffer is full and tell which buffer
     is active. */
  if( isr_current_geodata_index == NUMBER_OF_GEODATA_SAMPLES )
  {
    geodata_samples_real     = &geodata_samples[ 0 ];
    geodata_buffer_full      = true;
  }
  else if( isr_current_geodata_index == NUMBER_OF_GEODATA_SAMPLES * 2 )
  {
    geodata_samples_real      = &geodata_samples[ NUMBER_OF_GEODATA_SAMPLES ];
    isr_current_geodata_index = 0;
    geodata_buffer_full       = true;
  }

  /* In the same interrupt routine, handle report LED blinking. */
  report_blink( REPORT_BLINK_ENABLED );
}



/**
 * Blink the report LED if it has been enabled.
 *
 * @param enabled @a true if report blinking has been enabled.
 */
void report_blink( bool enabled )
{
  static unsigned long timestamp;
  static bool          led_on = false;

  if( enabled == true )
  {
    /* Turn on the LED and start a timer if a report was created. */
    if( report_was_created == true )
    {
      report_was_created = false;
      timestamp = millis( ) + 50;
      digitalWrite( REPORT_BLINK_LED_PIN, HIGH );
      led_on = true;
    }
    /* Turn off the LED once the timer expires. */
    if( led_on == true )
    {
      if( millis( ) > timestamp )
      {
        digitalWrite( REPORT_BLINK_LED_PIN, LOW );
        led_on = false;
      }
    }
  }
}



/**
 * Send the samples in the most recent buffer over the serial port.
 *
 * @param [in] freq_real Array of samples.
 * @param [in] length Number of samples.
 */
void report( const short *samples, int length )
{
  /* Send all the samples in the buffer to the serial port. */
  for( int index = 0; index < length; index++ )
  {
    Serial.print( " " );
	Serial.print( samples[ index ] );
  }
  /* Indicate to the report LED blinking that the report was submitted. */
  report_was_created = true;
}




/**
 * Initialize the serial port, setup the sampling, and turn off the on-board
 * LED.
 */
void setup()
{
  /* Initialize the serial port with the desired speed. */
  Serial.begin( SERIAL_SPEED );

  /* Setup the geophone data sampling buffers and sampling interrupt. */
  start_sampling( );

  /* Turn off the on-board LED. */
  pinMode( LED_PIN, OUTPUT );
  digitalWrite( LED_PIN, LOW );

  /* Configure the report LED if enabled. */
  report_was_created = false;
  if( REPORT_BLINK_ENABLED )
  {
    pinMode( REPORT_BLINK_LED_PIN, OUTPUT );
    digitalWrite( REPORT_BLINK_LED_PIN, LOW );
  }
}



/**
 * Main program loop which reports the samples every time the sample buffer
 * has been filled.
 */
void loop()
{
  /* Analyze the geophone data once it's available. */
  if( geodata_buffer_full == true )
  {
    geodata_buffer_full = false;

    /* Transmit the samples over the serial port. */
    report( geodata_samples, NUMBER_OF_GEODATA_SAMPLES );
  }
}

