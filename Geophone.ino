/****************************************************************************
 * geophone.ino
 *
 * Record geophone data and create a time vs. frequency profile.
 * The geophone data is sampled from the amplifier board and analyzed for'
 * frequency content by removing any frequency components whose magnitude
 * is below a threshold value.
 *
 * The analysis is provided as a time stamp (in plain text) followed by
 * binary frequency component pairs.  Each frequency component pair consists
 * of the frequency bin (0 through 255) and its corresponding amplitude
 * (-32767 through 32767).  An "XBee mode" setting changes the report to
 * a binary format of (split) packets for Digi's XBee API.
 *
 * The software requires an Arduino with at least 4096 bytes of RAM such as
 * the Arduino Mega or the Arduino Due.
 *
 * See the file COPYRIGHT for copyright details.  In addition, the bit-
 * reversal function is based on "Katja's homepage on sinusoids,
 * complex numbers and modulation" at <http://www.katjaas.nl/home/home.html>.
 ***************************************************************************/

#include <math.h>
#if defined( ARDUINO_AVR_MEGA2560 )
#include <EEPROM.h>
#endif



/* The geophone data is sampled on analog pin 5. */
#define GEODATA_PIN          5
/* Value for zero input, i.e., DC value, at the analog input. */
#define GEODATA_DC_OFFSET    159

/* Enable a human-readable report on the serial port. */
#define HUMAN_READABLE_REPORT_ENABLED  1

/* Select the serial port (Serial, Serial1, Serial2, etc.).  The plain
   serial port is for outputting the readable format of the report. */
#define SERIAL_PORT        Serial
/* Serial speed for the report generation.  It should be fast enough to
   allow several values to be passed per second.  A speed of 38,400 baud
   should suffice for worst case reports of about 2,300 bytes. */
#define SERIAL_SPEED       115200

/* If XBee is enabled, then a report is transmitted over the XBEE_SERIAL_PORT
   in a binary format in multiple packets with the following slightly
   compressed format:

   [ ReportID  { Frequency0 Amplitude0 }..{ FrequencyN AmplitudeN }  ]
      4              8         12       ..       8         12          (bits)

   Identical IDs indicate that packets are part of the same report. The end
   of a report is indicated by a frequency of 0 and no amplitude. */
#define XBEE_ENABLED        0
/* The XBee serial port is for outputting the report in a packed format
   and embedded in an XBee API frame. The serial port uses two stop bits
   because I recall reading this is necessary at speeds of 115200 baud and
   above. */
#define XBEE_SERIAL_PORT    Serial1
#define XBEE_SERIAL_SPEED   115200
#define XBEE_SERIAL_CONFIG  SERIAL_8N2
/* Pin used to reset the XBee at power-up because that appears to be
   necessary.  */
#define XBEE_RESET_PIN      51

/* XBee destination address for the report; in this case the XBee coordinator. */
#define XBEE_DESTINATION_ADDRESS_64  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
#define XBEE_DESTINATION_ADDRESS_16  { 0xff, 0xfe }
/* An XBee payload size is 84, or 66 if the message is encrypted.  (Otherwise
   the payload will be fragmented but let's stay in control.) */
#define MAX_XBEE_PAYLOAD_SIZE        66
#define MAX_XBEE_FRAME_SIZE          110
/* Define a delay in milliseconds between packets sent to the XBee radio so that
   its serial port gets time to flush. */
#define XBEE_TRANSMIT_DELAY 100

/* Report only frequency components within the range of the geophone sensor
   and the amplifier's bandpass setting.  For example with an SM-24 geophone
   measuring the range between 10 Hz to 240 Hz and an amplifier with a band-
   pass filter from 7 Hz to 150 Hz, the range is 10 Hz to 150 Hz. */
#define LOWEST_FREQUENCY_REPORTED    10
#define HIGHEST_FREQUENCY_REPORTED  150

/* Make an LED blink on every successful report. The on-board LED may be used
   but in this case it's pin 52. */
#define REPORT_BLINK_ENABLED    1
#define REPORT_BLINK_LED_PIN   52

/* Default threshold for reporting amplitudes. */
#define DEFAULT_AMPLITUDE_THRESHOLD  0.1

/* Define the geophone data sampling rate. */
#define SAMPLE_RATE   512

/* Include bit-reversed twiddle factors for a 512-point radix-2 DIT FFT. */
#define NUMBER_OF_GEODATA_SAMPLES 512
#include "twiddle_factors_256_br.h"

/* Define the on-board LED so we can turn it off. */
#define LED_PIN             13

/* EEPROM address where the amplitude threshold is stored.  Sadly, the Arduino
   Due doesn't have EEPROM. */
#define AMPLITUDE_THRESHOLD_EEPROM_ADDRESS 0


/* Create a double buffer for geodata samples so that the frequency analysis
 * may be performed on one buffer while the other is being filled with samples.
 * The imaginary part requires only one buffer.
 */
short geodata_samples[ NUMBER_OF_GEODATA_SAMPLES * 2 ];
short *geodata_samples_real;
short geodata_samples_imag[ NUMBER_OF_GEODATA_SAMPLES ];
/* Indexes used by the interrupt service routine. */
int  isr_hamming_window_index;
int  isr_current_geodata_index;
/* Semaphor indicating that a frame of geophone samples is ready. */
bool geodata_buffer_full;
/* Current threshold at which amplitudes are reported. */
double amplitude_threshold;
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
  isr_hamming_window_index  = 0;
  isr_current_geodata_index = 0;
  geodata_buffer_full       = false;

  /* Setup interrupts for the Arduino Mega. */
#if defined( ARDUINO_AVR_MEGA2560 )
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
  uint32_t rc = VARIANT_MCK / 128 / SAMPLE_RATE;
  /* Setup the timer. */
  TC_SetRA( TC1, 0, rc / 2 );
  TC_SetRC( TC1, 0, rc );
  TC_Start( TC1, 0 );
  TC1->TC_CHANNEL[ 0 ].TC_IER = TC_IER_CPCS;
  TC1->TC_CHANNEL[ 0 ].TC_IDR = ~TC_IER_CPCS;
  NVIC_EnableIRQ( TC3_IRQn );
#else
#error Arduino board not supported by this software.
#endif
}



#if defined( ARDUINO_AVR_MEGA2560 )
/**
 * Interrupt service routine for Arduino Mega devices which invokes the
 * generic interrupt service routine.
 */
ISR( TIMER1_COMPA_vect )
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
  /* Read a sample and store it in the geodata buffer.  Apply a Hamming
     window as we go along.  It involves a cos operation; the alternative
     is an array that should be fit into program memory. */
#if defined( ARDUINO_AVR_MEGA2560 )
  const int adc_resolution = 1024;
#elif defined( ARDUINO_SAM_DUE )
  const int adc_resolution = 4096;
#endif

  /* Read the geodata sample from the analog input. */
  short geodata_sample = analogRead( GEODATA_PIN )
                         - ( adc_resolution >> 1 ) - GEODATA_DC_OFFSET;
  /* Compute the Hamming window weight. */
  const double alpha = 0.54;
  const double beta  = 1.0 - alpha;
  const double N     = (double)( NUMBER_OF_GEODATA_SAMPLES - 1 );
  double hamming_window =
    alpha - beta * cos( 2.0 * M_PI * (double)isr_hamming_window_index / N );
  isr_hamming_window_index++;
  /* Scale the sample and apply the window weight. */
  double window = hamming_window;
  const int scale = window * 32768.0 / adc_resolution;
  geodata_sample = (short)( (double)geodata_sample * (double)scale );
  geodata_samples[ isr_current_geodata_index++ ] = geodata_sample;

  /* Raise a semaphor if the buffer is full and tell which buffer
     is active. */
  if ( isr_current_geodata_index == NUMBER_OF_GEODATA_SAMPLES )
  {
    /* The lower buffer has been filled. */
    geodata_samples_real     = &geodata_samples[ 0 ];
    isr_hamming_window_index = 0;
    geodata_buffer_full      = true;
  }
  else if ( isr_current_geodata_index == NUMBER_OF_GEODATA_SAMPLES * 2 )
  {
    /* The upper buffer has been filled. */
    geodata_samples_real      = &geodata_samples[ NUMBER_OF_GEODATA_SAMPLES ];
    isr_current_geodata_index = 0;
    isr_hamming_window_index  = 0;
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

  if ( enabled == true )
  {
    /* Turn on the LED and start a timer if a report was created. */
    if ( report_was_created == true )
    {
      report_was_created = false;
      /* Make it a rather short blink of just 75 ms. */
      timestamp = millis( ) + 75;
      digitalWrite( REPORT_BLINK_LED_PIN, HIGH );
      led_on = true;
    }
    /* Turn off the LED once the timer expires. */
    if ( led_on == true )
    {
      if ( millis( ) > timestamp )
      {
        digitalWrite( REPORT_BLINK_LED_PIN, LOW );
        led_on = false;
      }
    }
  }
}



/**
 * Compute the amplitude based on a real and an imaginary component.
 *
 * @param [in] real Real component.
 * @param [in] real Imaginary component.
 * @return Amplitude.
 */
double compute_amplitude( double real, double imaginary )
{
  double amplitude = sqrt( real * real + imaginary * imaginary );
  return ( amplitude );
}



/**
 * Swap two complex numbers in an array.
 *
 * @param [in] first Index of the first number.
 * @param [in] second Index of the other number.
 * @param [in,out] real Array of real components.
 * @param [in,out] imaginary Array of imaginary components.
 */
void swap( int first, int second, short *real, short *imaginary )
{
  short temp_r = real[ first ];
  real[ first ] = real[ second ];
  real[ second ] = temp_r;

  short temp_i = imaginary[ first ];
  imaginary[ first ] = imaginary[ second ];
  imaginary[ second ] = temp_i;
}



/**
 * Bit-reverse the elements in a complex array.
 *
 * @param [in,out] real Array of real components.
 * @param [in,out] imag Array of imaginary components.
 * @param [in] length Length of the array.
 */
void bit_reverse_complex( short *real, short *imag, int length )
{
  int N = length;

  int halfn  = N >> 1;
  int quartn = N >> 2;
  int nmin1  = N - 1;

  unsigned int forward = halfn;
  unsigned int rev     = 1;

  /* Start of bit-reversed permutation loop, N/4 iterations. */
  for ( int i = quartn; i; i-- )
  {
    /* Gray code generator for even values. */
    unsigned int nodd = ~i;                  // counting ones is easier
    int zeros;
    for ( zeros = 0; nodd & 1; zeros++ )
    {
      nodd >>= 1;   // find trailing zeros in i
    }
    forward ^= 2 << zeros;      // toggle one bit of forward
    rev     ^= quartn >> zeros; // toggle one bit of rev

    /* Swap even and ~even conditionally. */
    if ( forward < rev )
    {
      swap( forward, rev, real, imag );
      nodd = nmin1 ^ forward; // compute the bitwise negations
      unsigned int noddrev = nmin1 ^ rev;
      swap( nodd, noddrev, real, imag ); // swap bitwise-negated pairs
    }

    nodd = forward ^ 1;    // compute the odd values from the even
    unsigned int noddrev = rev ^ halfn;
    swap( nodd, noddrev, real, imag );  // swap odd unconditionally
  }
}



/**
 * Q.15  fractional integer 16-bit x 16-bit -> 16-bit multiplication.
 *
 * @param [in] a The first q.15 fractional integer factor.
 * @param [in] b The other q.15 fractional integer factor.
 * @return Product in q.15 fractional integer format.
 */
inline short q15_mul16( short a, short b )
{
  // Perform a 32-bit x 32-bit -> 32-bit integer multiplication.
  int32_t product = (int32_t)a * (int32_t)b;
  // Convert to q.15 fractional.
  return ( (short)( ( product >> 15 ) & 0x0000ffff ) );
}



/**
 * Complex addition using q.15 fractional arithmetic.
 *
 * @param [out] res_r Real part of complex sum.
 * @param [out] res_i Imaginary part of complex sum.
 * @param [in] r1 Real part of first complex number.
 * @param [in] i1 Imaginary part of first complex number.
 * @param [in] r2 Real part of second complex number.
 * @param [in] i2 Imaginary part of second complex number.
 */
inline void complex_add( short *res_r, short *res_i,
                         short r1, short i1, short r2, short i2 )
{
  *res_r = r1 + r2;
  *res_i = i1 + i2;
}



/**
 * Complex multiplication using q.15 fractional arithmetic.
 *
 * @param [out] res_r Real part of complex product.
 * @param [out] res_i Imaginary part of complex product.
 * @param [in] r1 Real part of first complex factor.
 * @param [in] i1 Imaginary part of first complex factor.
 * @param [in] r2 Real part of second complex factor.
 * @param [in] i2 Imaginary part of second complex factor.
 */
inline void complex_mul( short *res_r, short *res_i,
                         short r1, short i1, short r2, short i2 )
{
  *res_r = q15_mul16( r1, r2 ) - q15_mul16( i1, i2 );
  *res_i = q15_mul16( r1, i2 ) + q15_mul16( i1, r2 );
}



/**
 * Perform a 512-point radix-2 in-place DIT FFT using q.15 fractional
 * arithmetic.
 *
 * @param [in,out] data_real Array of real components of complex input/output.
 * @param [in,out] data_imag Array of imaginary components of complex
 *        input/output.
 */
void fft_radix2_512( short *data_real, short *data_imag )
{
  const int length = 512;

  int pairs_per_group  = length / 2;
  int wingspan         = length / 2;

  /* Divide and conquer. */
  for ( int stage = 1; stage < length; )
  {
    for ( int group = 0; group < stage; group++ )
    {
      /* Read the twiddle factors for the curent group. */
      short WR = pgm_read_word_near( twiddle_real + group );
      short WI = pgm_read_word_near( twiddle_imag + group );

      /* Calculate the positions of the butterflies in this group. */
      int lower = 2 * group * pairs_per_group;
      int upper = lower + pairs_per_group;

      /* Compute all the butterflies in the current group. */
      for ( int butterfly = lower; butterfly < upper; butterfly++ )
      {
        /* Compute one FFT butterfly. */
        short temp_r, temp_i;
        complex_mul( &temp_r, &temp_i, WR, WI,
                     data_real[ butterfly + wingspan ],
                     data_imag[ butterfly + wingspan ]);
        complex_add( &data_real[ butterfly + wingspan ],
                     &data_imag[ butterfly + wingspan ],
                     data_real[ butterfly ], data_imag[ butterfly ],
                     -temp_r, -temp_i );
        complex_add( &data_real[ butterfly ], &data_imag[ butterfly ],
                     data_real[ butterfly ], data_imag[ butterfly ],
                     temp_r, temp_i );
      }
    }

    pairs_per_group = pairs_per_group >> 1;
    stage           = stage << 1;
    wingspan        = wingspan >> 1;
  }
}



/**
 * Transmit a package via an XBee device in API mode.  The package
 * is embedded in a Transmit Request (type 0x10) frame.
 *
 * @param [in] payload Package to send via XBee.
 * @param [in] payload_size Number of bytes in the package.
 * @param [in] address_64 64-bit XBee destination address.
 * @param [in] address_16 16-bit XBee destination address.
 */
void transmit_xbee_payload( const unsigned char *payload,
                            int payload_size,
                            const unsigned char *address_64,
                            const unsigned char *address_16 )
{
  static unsigned char frame_id = 1;
  const int header_size = 1 + 2 + 1 + 1 + 8 + 2 + 1 + 1;
  unsigned char xbee_frame[ header_size ];
  int pos = 0;

  /* Compose the frame header for a transmit request (0x10). */
  xbee_frame[ pos++ ] = 0x7e;
  xbee_frame[ pos++ ] = ( ( header_size + payload_size - 3 ) >> 8 ) & 0x00ff;
  xbee_frame[ pos++ ] = ( header_size + payload_size - 3 ) & 0x00ff;
  xbee_frame[ pos++ ] = 0x10;
  /* Setup destination address and transmit options. */
  xbee_frame[ pos++ ] = frame_id;
  for ( int i = 0; i < 8; i++ )
  {
    xbee_frame[ pos++ ] = address_64[ i ];
  }
  xbee_frame[ pos++ ] = address_16[ 0 ];
  xbee_frame[ pos++ ] = address_16[ 1 ];
  /* Options:  retry/repair enabled, encryption enabled, no extended timeout. */
  xbee_frame[ pos++ ] = 0;
  xbee_frame[ pos++ ] = 0x20;

  /* Copy the payload header header to the XBee. */
  XBEE_SERIAL_PORT.write( xbee_frame, header_size );
  /* Copy the payload to the Xbee device. */
  XBEE_SERIAL_PORT.write( payload, payload_size );

  /* Compute the checksum for the payload header. */
  unsigned char checksum = 0x00;
  for ( int i = 3; i < header_size; i++ )
  {
    checksum += xbee_frame[ i ];
  }
  /* Compute the checksum for the payload. */
  for ( int i = 0; i < payload_size; i++ )
  {
    checksum += payload[ i ];
  }

  /* Copy the checksum to the XBee, completing the frame. */
  checksum = 0xff - checksum;
  XBEE_SERIAL_PORT.write( checksum );

  /* Advance to the next XBee frame. */
  frame_id = frame_id + 1;
  if ( frame_id == 0 )
  {
    frame_id = 1;
  }
}



/**
 * Append 8 bits in a buffer that is composed of 4-bit values.
 *
 * @param [in,out] buffer Buffer with 4-bit values.
 * @param [in] value The 8-bit value to append to the buffer.
 * @param [in] bitpos Current bit position in the buffer.
 * @return New bit position in the buffer.
 */
int append_8_bits( unsigned char *buffer, unsigned char value, int bitpos )
{
  /* Compute the byte position to insert the value. */
  int bytepos = bitpos >> 3;
  /* If byte-aligned, just copy the byte. */
  if ( bitpos == bytepos << 3 )
  {
    buffer[ bytepos ] = value;
  }
  else
  {
    unsigned char upper_nibble = ( value >> 4 ) & 0x0f;
    unsigned char lower_nibble = value << 4;
    buffer[ bytepos ]     |= upper_nibble;
    buffer[ bytepos + 1 ]  = lower_nibble;
  }
  return ( bitpos + 8 );
}



/**
 * Append 12 bits in a buffer that is composed of 4-bit values.
 *
 * @param [in,out] buffer Buffer with 4-bit values.
 * @param [in] value The 12-bit value to append to the buffer.
 * @param [in] bitpos Current bit position in the buffer.
 * @return New bit position in the buffer.
 */
int append_12_bits( unsigned char *buffer, unsigned short value, int bitpos )
{
  /* Compute the byte position to insert the value. */
  int bytepos = bitpos >> 3;
  /* If byte-aligned, copy the first byte and append the remaining nibble. */
  if ( bitpos == bytepos << 3 )
  {
    unsigned char first_byte    = (unsigned char)( ( value >> 4 ) & 0x00ff );
    unsigned char second_nibble = (unsigned char)( value & 0x0f ) << 4;
    buffer[ bytepos     ] = first_byte;
    buffer[ bytepos + 1 ] = second_nibble;
  }
  else
  {
    unsigned char first_nibble = (unsigned char)( value >> 8 );
    unsigned char second_byte  = (unsigned char)( value & 0x00ff );
    buffer[ bytepos     ] |= first_nibble;
    buffer[ bytepos + 1 ]  = second_byte;
  }

  return ( bitpos + 12 );
}



/**
 * Convert a floating-point value to a string.
 *
 * @param [in] value Floating-point value.
 * @param [out] string Destination string.
 * @param [in] decimals Number of digits after the decimal point.
 */
void ftoa( double value, char *string, int decimals )
{
  int integer     = (int)value;
  double fraction = value - (double)integer;

  double power = 1.0;
  for( int i = 0; i < decimals; i++ )
  {
    power = power * 10;
  }
  fraction = fraction * power + 0.5;
  int int_fraction = (int)fraction;
  
  sprintf( string, "%d.%d", integer, int_fraction );
}



/**
 * Write a frequency/amplitude pair to the serial port and calculate the checksum
 * for the string.
 *
 * @param [in] frequency Frequency bin.
 * @param [in] amplitude Amplitude for this frequency component.
 * @return Checksum of the "frequency,amplitude" string.
 */
unsigned char report_pair( short frequency, double amplitude )
{
  char output_string[ 20 ];
  unsigned char checksum = 0;

  /* Print "frequency,". */
  sprintf( output_string, "%d,", frequency );
  for( int i = 0; i < strlen( output_string ); i++ )
  {
    checksum = checksum + output_string[ i ];
  }
  SERIAL_PORT.print( output_string );

  /* Print "amplitude" with no more than four decimal places. */
  ftoa( amplitude, output_string, 4 );
  for( int i = 0; i < strlen( output_string ); i++ )
  {
    checksum = checksum + output_string[ i ];
  }
  SERIAL_PORT.print( output_string );

  return( checksum );
}

/**
 * Print an array of amplitudes that are above the specified threshold.
 *
 * @param [in] freq_real Array of real components of the frequency bins.
 * @param [in] freq_imag Array of imaginary components of the frequency bins.
 * @param [in] length Number of frequency bins.
 * @param [in] threshold Threshold for reporting an amplitude component.
 */
void report( const short *freq_real, const short *freq_imag, int length,
             double threshold )
{
  bool first_entry       = true;
  unsigned char checksum = 0;

#if XBEE_ENABLED != 0
  const unsigned char xbee_address_64[ ] = XBEE_DESTINATION_ADDRESS_64;
  const unsigned char xbee_address_16[ ] = XBEE_DESTINATION_ADDRESS_16;

  static unsigned char report_id = 0;
  unsigned char        binary_report[ MAX_XBEE_PAYLOAD_SIZE ];
  int                  binary_report_bitpos = 0;

  /* Make room for the report ID, an array of frequency/amplitude pairs,
     and a terminating zero byte. */
  const int max_pairs_per_packet = ( MAX_XBEE_PAYLOAD_SIZE - 2 ) / 3 - 1;
  int       pairs_in_packet      = 0;

  binary_report[ 0 ]   = report_id << 4;
  binary_report_bitpos = 4;
#endif

  /* Walk through the frequency components array and report any frequency
     whose amplitude is above the specified threshold. */
  const float frequency_ratio = (float)NUMBER_OF_GEODATA_SAMPLES / (float)SAMPLE_RATE;
  const int lower_frequency_bin = (int)( LOWEST_FREQUENCY_REPORTED * frequency_ratio );
  const float upper_frequency_bin = (int)( HIGHEST_FREQUENCY_REPORTED * frequency_ratio );
  for ( int frequency_bin = lower_frequency_bin;
        frequency_bin <= upper_frequency_bin;
        frequency_bin++ )
  {
    /* Compute the amplitude. */
    double real = (double)freq_real[ frequency_bin ] / 32768.0;
    double imag = (double)freq_imag[ frequency_bin ] / 32768.0;
    double amplitude = sqrt( real * real + imag * imag );
    /* Report the frequency bin and the amplitude if the threshold is
       exceeded. */
    if ( amplitude >= threshold )
    {
      /* Comma-separate the numbers. */
      if ( first_entry == true )
      {
        first_entry = false;
      }
      else
      {
#if HUMAN_READABLE_REPORT_ENABLED == 1
        SERIAL_PORT.print( "," );
        checksum = checksum + ',';
#endif
      }
      /* Print the frequency bin and its amplitude. */
      double frequency =
        (double)SAMPLE_RATE / (double)NUMBER_OF_GEODATA_SAMPLES
        * (double)frequency_bin + 0.5;
#if HUMAN_READABLE_REPORT_ENABLED == 1
      checksum = checksum + report_pair( (short)frequency, amplitude );
#endif

#if XBEE_ENABLED != 0
      /* Add the frequency component / amplitude pair to the binary report. */
      binary_report_bitpos = append_8_bits( binary_report,
                                            (unsigned char)frequency,
                                            binary_report_bitpos );
      binary_report_bitpos = append_12_bits( binary_report,
                                             (short)( amplitude * 4096.0 + 0.5),
                                             binary_report_bitpos );
      pairs_in_packet = pairs_in_packet + 1;
      /* Transmit the packet if the payload size is about to exceed maximum. */
      if ( pairs_in_packet > max_pairs_per_packet )
      {
        transmit_xbee_payload( binary_report, ( binary_report_bitpos + 7 ) >> 3,
                               xbee_address_64, xbee_address_16 );
        /* Allow the XBee to flush its serial port. */
        delay( XBEE_TRANSMIT_DELAY );
        /* Prepare the header for the next report. */
        binary_report[ 0 ]   = report_id << 4;
        binary_report_bitpos = 4;
        pairs_in_packet = 0;
      }
#endif
    }
  }
  /* Terminate the report if any output was reported and indicate to the
     report LED blinking that the report was submitted. */
  if ( first_entry == false )
  {
#if HUMAN_READABLE_REPORT_ENABLED == 1
    SERIAL_PORT.print( "," );
    SERIAL_PORT.println( checksum );
#endif
#if XBEE_ENABLED != 0
    /* Add a frequency component with the value 0 to indicate that this is
       the last entry for this report. */
    binary_report_bitpos = append_8_bits( binary_report, 0,
                                          binary_report_bitpos );
    transmit_xbee_payload( binary_report, ( binary_report_bitpos + 7 ) >> 3,
                           xbee_address_64, xbee_address_16 );

    report_id = report_id + 1;
#endif
    report_was_created = true;
  }
}



/**
 * Read the current threshold value from EEPROM.  Use the default value if no
 * value has been stored in EEPROM.
 *
 * @return Stored (or default) threshold value.
 */
double read_amplitude_threshold_from_eeprom( )
{
  double        threshold;
  unsigned char *threshold_bytes = (unsigned char*)&threshold;
  for ( int i = 0; i < sizeof( double ); i++ )
  {

#if defined( ARDUINO_AVR_MEGA2560 )
    byte value = EEPROM.read( AMPLITUDE_THRESHOLD_EEPROM_ADDRESS + i );
#else
    byte value = 0;
#endif
    threshold_bytes[ i ] = value;
  }
  if ( threshold == 0.0 )
  {
    threshold = DEFAULT_AMPLITUDE_THRESHOLD;
  }
  return ( threshold );
}



/**
 * Save a new threshold value to EEPROM.
 *
 * @param New threshold value to store in EEPROM.
 */
void save_amplitude_threshold_to_eeprom( double threshold )
{
#if defined( ARDUINO_AVR_MEGA2560 )
  unsigned char *threshold_bytes = (unsigned char*)&threshold;
  for ( int i = 0; i < sizeof( double ); i++ )
  {
    byte value = threshold_bytes[ i ];
    EEPROM.write( AMPLITUDE_THRESHOLD_EEPROM_ADDRESS + i, value );
  }
#endif
}



/**
 * Read an XBee frame from the serial port and verify the checksum.  This
 * function should be called regularly.
 *
 * @param [out] payload Buffer for the frame except the frame delimiter.
 * @param [out] source_address_64 64-bit source address.
 * @param [out] source_address_16 16-bit source address.
 * @return Total number of bytes in the frame (excluding the delimiter), or
 *          -1 if the checksum was invalid.  If 0 bytes are received, then the
 *         frame hasn't been received yet.
 */
int receive_xbee_frame( unsigned char *frame,
                        unsigned char *source_address_64,
                        unsigned char *source_address_16 )
{
  static int           position = 0;
  static int           reported_frame_size;
  static unsigned char checksum;

  /* Get a byte from the serial port, if any. */
  if ( XBEE_SERIAL_PORT.available( ) > 0 )
  {
    char incoming_byte = XBEE_SERIAL_PORT.read( );
    /* Wait for a frame delimiter. */
    if ( position == 0 )
    {
      if ( incoming_byte != 0x7e )
      {
        return ( 0 );
      }
      reported_frame_size = 3;
      checksum = 0xff;
    }

    /* Populate the frame. */
    frame[ position++ ] = incoming_byte;

    /* If the maximum frame size is exceeded, reset the frame and report
       error. */
    if ( position == MAX_XBEE_FRAME_SIZE )
    {
      position = 0;
      return ( -1 );
    }

    /* Read the reported size. */
    else if ( position == 2 )
    {
      reported_frame_size = frame[ 0 ] << 8 | frame[ 1 ];
    }
    else if ( position > 2 )
    {
      checksum += incoming_byte;
    }

    /* If the entire frame has been received, the last byte was the
       checksum. */
    if ( position == reported_frame_size )
    {
      /* Reset the frame and report a checksum error if the checksum is
         invalid. */
      if ( incoming_byte != checksum )
      {
        position = 0;
        return ( -1 );
      }
      /* The frame is good so return the reported frame size plus the size
         bytes and the checksum. */
      else
      {
        return ( reported_frame_size + 3 );
      }
    }
  }

  return ( 0 );
}



/**
 * Read a new threshold value via an XBee package received from the serial
 * port.  The function is invoked regularly to receive bytes one at a time.
 * If a proper value is received return it.  Otherwise return a negative
 * value, which is an invalid amplitude.
 *
 * The function responds with "OK" if the number was successfully read or
 * provides a brief error message otherwise.
 *
 * @return New threshold value, or negative if no new value was provided.
 */
double get_new_threshold_xbee_mode( )
{
  static unsigned char xbee_frame[ MAX_XBEE_FRAME_SIZE ];
  unsigned char source_address_64[ 8 ];
  unsigned char source_address_16[ 2 ];

  /* Receive an XBee frame, if any. */
  int frame_size = receive_xbee_frame( xbee_frame,
                                       source_address_64, source_address_16 );
  if ( frame_size > 14 )
  {
    /* Verify that it's a receive data frame. */
    if ( xbee_frame[ 2 ] == 0x90 )
    {
      /* Zero-terminate the frame content because it's a string. */
      xbee_frame[ frame_size - 1 ] = '\0';
      /* Read the threshold value. */
      double new_threshold = atof( (const char*)&xbee_frame[ 14 ] );
      /* Return the new threshold value if it is valid. */
      if ( new_threshold >= 0.0 && new_threshold < 1.0 )
      {
        return ( new_threshold );
      }
    }
  }
  return ( -1.0 );
}



/**
 * Flush the serial input buffer.
 */
void flush_serial_input( )
{
  while ( SERIAL_PORT.available( ) > 0 )
  {
    SERIAL_PORT.read( );
  }
}



/**
 * Read a new threshold value from the serial port.  The function is invoked
 * regularly and reads a byte from the serial port, if any.  If a proper
 * value is read (within a short time to avoid spurious bytes eventually
 * forming a value), return it.  Otherwise return a negative value, which is
 * an invalid amplitude.
 *
 * The function responds with "OK" if the number was successfully read or
 * provides a brief error message otherwise.
 *
 * If XBee is enabled, the new threshold is read from the XBee serial port
 * instead via the function get_new_threshold_xbee_mode().
 *
 * @return New threshold value, or negative if no new value was provided.
 */
double get_new_threshold( )
{
#if HUMAN_READABLE_REPORT_ENABLED != 0
  static char          threshold_string[ 20 ];
  static int           threshold_string_pos = 0;
  static unsigned long timestamp;
  /* Timeout is 250 ms. */
  const unsigned long  timeout = 250;

  /* Timeout if characters are not being received swiftly enough. */
  if ( ( threshold_string_pos > 0 ) && ( timestamp + timeout < millis( ) ) )
  {
    SERIAL_PORT.println( "E:TIMEOUT" );
    flush_serial_input( );
    threshold_string_pos = 0;
  }

  /* Read the next byte from the serial port, if any. */
  else if ( SERIAL_PORT.available( ) > 0 )
  {
    /* The first byte should start a timeout counter.  The entire string
       is discarded if the timer expires before the value has been
       submitted to the device. */
    if ( threshold_string_pos == 0 )
    {
      timestamp = millis( );
    }

    if ( threshold_string_pos < sizeof( threshold_string ) - 1 )
    {
      char incoming_byte = SERIAL_PORT.read( );
      /* Attempt to parse the new threshold value once a newline is received. */
      if ( incoming_byte == '\r' || incoming_byte == '\n' )
      {
        threshold_string[ threshold_string_pos ] = '\0';
        double threshold = atof( threshold_string );
        /* Only threshold values between 0.0 and 1.0 are allowed. */
        if ( threshold < 0.0 || threshold >= 1.0 )
        {
          threshold = -1.0;
          SERIAL_PORT.println( "E:OVERFLOW" );
        }
        /* The threshold value is valid so report success. */
        else
        {
          SERIAL_PORT.print( "OK:" );
          SERIAL_PORT.println( threshold_string );
        }
        flush_serial_input( );
        threshold_string_pos = 0;
        return( threshold );
      }
      /* Add the byte to the threshold string if it's a valid numerical
         character. */
      else if (    ( ( incoming_byte >= '0' ) && ( incoming_byte <= '9' ) )
                   || ( incoming_byte == '.' ) )
      {
        threshold_string[ threshold_string_pos++ ] = incoming_byte;
      }
      /* Any invalid byte resets the string. */
      else
      {
        SERIAL_PORT.println( "E:NON-NUMERIC" );
        flush_serial_input( );
        threshold_string_pos = 0;
      }
    }
    /* Too much junk in the input buffer; discard everything. */
    else
    {
      SERIAL_PORT.println( "E:FLUSH" );
      flush_serial_input( );
      threshold_string_pos = 0;
    }
  }
#endif

#if XBEE_ENABLED
  return ( get_new_threshold_xbee_mode( ) );
#endif

  return ( -1.0 );
}



/**
 * Initialize the amplitude threshold from last run and initialize the
 * serial port.  Also, turn off the on-board LED.
 */
void setup()
{
  /* Read the amplitude threshold value from EEPROM or use the default value
     if nothing was stored in EEPROM. */
  amplitude_threshold = read_amplitude_threshold_from_eeprom( );

  /* Initialize the serial port with the desired speed. */
  SERIAL_PORT.begin( SERIAL_SPEED );

#if XBEE_ENABLED != 0
  /* Reset the XBee because for some reason that's necessary. But actually,
     even this code doesn't cut it. The XBee needs to be reset from the
     coordinator via a remote AT command. Must look into this. */
/*
  delay( 250 );
  pinMode( XBEE_RESET_PIN, OUTPUT );
  digitalWrite( XBEE_RESET_PIN, LOW );
  delay( 50 );
  digitalWrite( XBEE_RESET_PIN, HIGH );
*/
  /* Initialize the XBee serial port. */
  XBEE_SERIAL_PORT.begin( XBEE_SERIAL_SPEED, XBEE_SERIAL_CONFIG );
#endif

  /* Setup the analog input. */
  analogReference( DEFAULT );
  pinMode( GEODATA_PIN, INPUT );
  /* Setup the geophone data sampling buffers and sampling interrupt. */
  start_sampling( );

  /* Turn off the on-board LED. */
  pinMode( LED_PIN, OUTPUT );
  digitalWrite( LED_PIN, LOW );

  /* Configure the report LED if enabled. */
  report_was_created = false;
  if ( REPORT_BLINK_ENABLED )
  {
    pinMode( REPORT_BLINK_LED_PIN, OUTPUT );
    digitalWrite( REPORT_BLINK_LED_PIN, LOW );
  }
}



/**
 * Main program loop which performs a frequency analysis each time the
 * geophone sample buffer has been filled and creates a report with the
 * frequency/amplitude data.  The main loop also listens for new amplitude
 * threshold values submitted via the serial port.
 */
void loop()
{
  /* Analyze the geophone data once it's available. */
  if ( geodata_buffer_full == true )
  {
    /* Set the imaginary data values to 0 as this is a real-valued signal. */
    unsigned long *geodata_samples_imag_l = (unsigned long *)&geodata_samples_imag[ 0 ];
    for( int i = 0; i < NUMBER_OF_GEODATA_SAMPLES >> 1; i++ )
    {
      geodata_samples_imag_l[ i ] = 0ul;
    }
    /* Compute the Fourier transform in-place. */
    fft_radix2_512( geodata_samples_real, &geodata_samples_imag[ 0 ] );
    bit_reverse_complex( geodata_samples_real, &geodata_samples_imag[ 0 ],
                         NUMBER_OF_GEODATA_SAMPLES );
     /* Compute the amplitudes and report them in the same run.  Since
       the input data is real, the last half of the Fourier transform is
       is the complex conjugate of the first half and thus redundant.
       (This is why we bother to bit-reverse the Fourier-transformed
       data:  otherwise the redundant information would be spread out
       in the output data and we would have to use bit-reversed addressing
       to pick the non-redundant values from the array anyway.) */
    report( geodata_samples_real, &geodata_samples_imag[ 0 ],
            NUMBER_OF_GEODATA_SAMPLES / 2 + 1, amplitude_threshold );

    geodata_buffer_full = false;
  }

  /* Read any new threshold value that may be provided.  Update the threshold
     and write it to EEPROM if the value is valid. */
  double threshold = get_new_threshold( );
  if ( threshold > 0.0 )
  {
    save_amplitude_threshold_to_eeprom( threshold );
    amplitude_threshold = threshold;
  }
}

