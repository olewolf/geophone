#!/usr/bin/perl
#
# Filter log entries between <fromdate> and <todate>
#
# USAGE:  filter-log.pl <fromdate> <todate>
#
# The script reads from stdin and writes to stdout.  The dates must
# have the format:  Sat Feb 14 16:55:28 2015.


use strict;
use Time::Piece;
#use Date::Parse;
use Data::Dumper qw(Dumper);


sub usage
{
	print "USAGE:  filter-log.pl <fromdate> <todate>\n";
	exit;
}

# Get the complete argument string.
my $arguments = '';
foreach (@ARGV) {
    $arguments .= /\s/ ? " \'" . $_ . "\'" : " "   . $_;
}

# Read the two dates specified on the command line.
$arguments =~ /([a-zA-Z]{3} [a-zA-Z]{3} {1,2}[0-9]{1,2} [0-9]{2}:[0-9]{2}:[0-9]{2} [0-9]{4}) ([a-zA-Z]{3} [a-zA-Z]{3} {1,2}[0-9]{1,2} [0-9]{2}:[0-9]{2}:[0-9]{2} [0-9]{4}).*/;
my( $fromdate, $todate ) = ( $1, $2 );
if( $fromdate eq "" )
{
	usage;
}

# Convert timestamps to UNIX time.
my $from_timestamp = str2time( $fromdate );
my $to_timestamp   = str2time( $todate );

foreach my $log_entry( <STDIN> )
{
	chomp $log_entry;

	my $log_entry_begin = substr $log_entry, 0, 2;
	if( $log_entry_begin ne "E:" && $log_entry_begin ne "OK" )
	{
		my $log_timestamp;
		my $measurements;
		my @test = $log_entry =~ /([a-zA-Z]{3} [a-zA-Z]{3} {1,2}[0-9]{1,2} [0-9]{2}:[0-9]{2}:[0-9]{2} [0-9]{4}) (.*)/;
		if( $#test == 1 )
		{
			$log_timestamp = $test[ 0 ];
			$measurements  = $test[ 1 ];

			if( $log_timestamp ne "" )
			{
				# Convert the text timestamp to UNIX time.
#				$log_timestamp = str2time( $log_timestamp );
				my $time_piece = Time::Piece->strptime( $log_timestamp, "%c" );
				$log_timestamp = $time_piece->epoch;
				
				if( ( $from_timestamp <= $log_timestamp ) && ( $log_timestamp <= $to_timestamp  ) )
				{
					print "$log_entry\n";
				}
			}
		}
	}
}

