/**
 * Example code that reads frames from the geophone from the serial port
 * and writes the frames together with a timestamp if the checksum is valid.
 */


#define _GNU_SOURCE

#include <glib-2.0/glib.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>


typedef struct
{
	gchar *frame;
	int   size;
} frame_t;



/**
 * Determine if the sum of bytes matches the last value in the frame, and
 * NULL-terminate the string where the checksum began.
 */
bool check_checksum ( char *frame )
{
	unsigned char checksum = 0;
	unsigned char reported_checksum = 0;

	/* Find the last ',' in the string. */
	char *last_comma = &frame[ strlen( frame ) ];
	while( last_comma >= frame )
	{
		if( *last_comma == ',' )
		{
			break;
		}
		else
		{
			last_comma--;
		}
	}

	if( *last_comma == ',' )
	{
		const char *checksum_pos = &last_comma[ 1 ];
		reported_checksum = atoi( checksum_pos );

		/* Compute the checksum of the received string. */
		for( int pos = 0; frame < last_comma; frame++ )
		{
			checksum = checksum + frame[ pos ];
		}

		if( checksum == reported_checksum )
		{
			*last_comma = '\0';
			return( true );
		}
	}

	fprintf( stderr, "Invalid checksum %d vs. reported %d\n", checksum, reported_checksum );
	return( false );
}



/**
 * Read bytes from the serial port until a newline is received.
 */
static frame_t *read_frame( int serial )
{
	gchar *frame             = g_malloc( 16384 );
	int   amount_allocated   = 16384;
	int   pos                = 0;
	int   checksum_pos_begin = 0;

	time_t    timestamp = 0;
	struct tm time_result;

	unsigned char ch = 0;
	do
	{
		int bytes_read;
		do
		{
			bytes_read = read( serial, &ch, 1 );
		} while( bytes_read < 1 );
		if( timestamp == 0 )
		{
			timestamp = time( NULL );
			localtime_r( &timestamp, &time_result );
			asctime_r( &time_result, &frame[ pos ] );
			pos = pos + strlen( frame ) - 1;
			frame[ pos++ ] = ' ';
			checksum_pos_begin = pos;
		}

		if( ch != '\n' && ch != '\r' )
		{
			frame[ pos++ ] = ch;
		}
		else
		{
			frame[ pos++ ] = '\0';
		}
		/* Dynamically expand the buffer as needed. */
		if( pos == amount_allocated )
		{
			frame = g_realloc( frame, amount_allocated + 4096 );
			amount_allocated += 4096;
		}
	} while( ch != '\n' );



	/* Check the checksum. */
	if( check_checksum( &frame[ checksum_pos_begin ] ) == true )
	{
		frame_t *frame_info = g_new( frame_t, 1 );
		frame_info->size  = pos - 1;
		frame_info->frame = frame;
		return( frame_info );
	}
	else
	{
		g_free( frame );
		return( NULL );
	}
}



int main( int argc, char *argv[ ] )
{
	if( argc != 2 )
	{
		fprintf( stderr, "USAGE: %s <serial port>\n", argv[ 0 ] );
		exit( EXIT_FAILURE );
	}

	int serial = open( argv[ 1 ], O_RDWR| O_NOCTTY );
	struct termios tty;
	bzero( &tty, sizeof( tty ) );
	if ( tcgetattr ( serial, &tty ) != 0 )
	{
		fprintf( stderr, "Can't open serial port %s: ", argv[ 1 ] );
		perror( "" );
		exit( EXIT_FAILURE );
	}

	/* Set Baud Rate */
	cfsetospeed( &tty, B115200 );
	cfsetispeed( &tty, B115200 );

	/* Setting other Port Stuff */
	tty.c_cflag     &=  ~PARENB; 
	tty.c_cflag     &=  ~CSTOPB;
	tty.c_cflag     &=  ~CSIZE;
	tty.c_cflag     |=  CS8;
	tty.c_cflag     &=  ~CRTSCTS;
	tty.c_cc[VMIN]   =  1;
	tty.c_cc[VTIME]  =  5;
	tty.c_cflag     |=  CREAD | CLOCAL;

	/* Make raw */
	cfmakeraw( &tty );

	/* Flush port, then apply attributes */
	tcflush( serial, TCIFLUSH );
	if ( tcsetattr ( serial, TCSANOW, &tty ) != 0)
	{
		perror( "Can't set serial port attributes: " );
		exit( EXIT_FAILURE );
	}

	/* Make the output unbuffered. */
	//setbuf( out_fhd, NULL );
	FILE *out_fhd = stdout;
	setvbuf( out_fhd, NULL, _IONBF, 0 );

	while( 1 )
	{
		frame_t *frame = read_frame( serial );

		if( frame != NULL )
		{
			printf( "%s\n", frame->frame );
			fflush( out_fhd );

			g_free( frame->frame );
			g_free( frame );
		}
	}

	exit( EXIT_SUCCESS );
}
