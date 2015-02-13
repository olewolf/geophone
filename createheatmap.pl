#!/usr/bin/perl
#
# Create a heatmap based on the frequencies and amplitudes versus
# time read from a log file.

use strict;
use Image::Magick;
#use Data::Dumper qw(Dumper);

#my $desired_colormap = "hot";
my $desired_colormap = "spectral";


my %color_maps = (
	"Accent" => 16,
	"Blues" => 30,
	"BrBG" => 49,
	"BuGn" => 68,
	"BuPu" => 83,
	"Dark2" => 98,
	"GnBu" => 117,
	"Greens" => 134,
	"Greys" => 151,
	"OrRd" => 166,
	"Oranges" => 181,
	"PRGn" => 200,
	"Paired" => 216,
	"Pastel1" => 234,
	"Pastel2" => 250,
	"PiYG" => 265,
	"PuBu" => 285,
	"PuBuGn" => 300,
	"PuOr" => 316,
	"PuRd" => 333,
	"Purples" => 351,
	"RdBu" => 366,
	"RdGy" => 380,
	"RdPu" => 400,
	"RdYlBu" => 417,
	"RdYlGn" => 433,
	"Reds" => 449,
	"Set1" => 468,
	"Set2" => 483,
	"Set3" => 501,
	"Spectral" => 518,
	"YlGn" => 531,
	"YlGnBu" => 554,
	"YlOrBr" => 567,
	"YlOrRd" => 584,
	"autumn" => 601,
	"binary" => 616,
	"bone" => 634,
	"cool" => 651,
	"copper" => 666,
	"flag" => 682,
	"gist_earth" => 700,
	"gist_gray" => 716,
	"gist_heat" => 735,
	"gist_ncar" => 750,
	"gist_rainbow" => 764,
	"gist_stern" => 784,
	"gist_yarg" => 799,
	"gray" => 819,
	"hot" => 834,
	"hsv" => 848,
	"jet" => 867,
	"pink" => 884,
	"prism" => 899,
	"spectral" => 916,
	"spring" => 932,
	"summer" => 952,
	"winter" => 966
	);

# Read the log file.
open( LOG, "geophone.log" ) || die;
my @log = <LOG>;
close( LOG );

# Read the log, assuming for now that it doesn't contain errors.
# Create a report which is an array of triplets:  a timestamp, an array of
# frequencies, and an array of their corresponding amplitudes.
my @report_timestamps;
my @report_frequencies;
my @report_amplitudes;
my $report_index = 0;
my $timestamp = 0;
my $min_timestamp = 0;
my $max_timestamp = 0;
my $min_frequency = 0;
my $max_frequency = 0;
foreach my $log_entry( @log )
{
	my @frequencies;
	my @amplitudes;

	my $log_entry_begin = substr $log_entry, 0, 1;
	if( $log_entry_begin != 'E' && $log_entry_begin != "O" )
	{
		# If the log entry contains frequency/amplitude pairs, put a timestamp
		# into the report.
		$report_timestamps[ $report_index ] = $timestamp;
		if( $timestamp < $min_timestamp )
		{
			$min_timestamp = $timestamp;
		}
		if( $timestamp > $max_timestamp )
		{
			$max_timestamp = $timestamp;
		}
		$timestamp = $timestamp + 1;
		# Read the frequencies and the amplitudes.
		my @values = split /,/, $log_entry;
		my $pairs_reported = ( $#values + 1 ) / 2;
		for( my $pair = 0; $pair < $pairs_reported; $pair++ )
		{
			my $frequency = 0 + "@values[ $pair * 2 ]";
			if( $frequency < $min_frequency )
			{
				$min_frequency = $frequency;
			}
			if( $frequency > $max_frequency )
			{
				$max_frequency = $frequency;
			}
			$frequencies[ $pair ] = $frequency;
			$amplitudes[ $pair ]  = 0 + "@values[ $pair * 2 + 1 ]";
		}
		# Add the frequencies and amplitudes to the report.
		$report_frequencies[ $report_index ] = join( ",", @frequencies );
		$report_amplitudes[ $report_index ]  = join( ",", @amplitudes );
		$report_index = $report_index + 1;
	}
}

# Create an array of color names based on the heatmap colors.
my $heatmap = Image::Magick->new( );
$heatmap->Read( "colormaps.png" );
my $heatmap_length = 352;
my $bar_y_pos = $color_maps{ "$desired_colormap" };
my @color_map;
for( my $i = 0; $i < $heatmap_length; $i++ )
{
	my @color = $heatmap->GetPixel( x => $i, y => $bar_y_pos );
	foreach my $x ( @color ) { $x = $x * 255;  }
	my $color_name = "rgb(" . join( ",", @color ) . ")";
	@color_map[ $i ] = $color_name;
}

# Set plotting parameters.
my $dot_width  = 4;
my $x_margin   = 10;
my $y_margin   = 5;
# Create an empty canvas.
my $coordinate_system = Image::Magick->new( );
my $plot_width = $x_margin * 2
	+ ( $max_timestamp - $min_timestamp + 1 ) * $dot_width;
$coordinate_system->Set( size => "$plot_width" . "x600" );
$coordinate_system->ReadImage( "canvas:black" );
# Compute the dot height based on the height of the image.
my $y_size = $coordinate_system->Get( "height" );
my $number_of_frequency_components = 256;
my $dot_height = int( ( $y_size - 2 * $y_margin ) / $number_of_frequency_components );
my $x_size = $coordinate_system->Get( "width" );

# Plot the dots, colored according to the color map.
my $entries = $#report_timestamps;
for( my $report_index = 0; $report_index < $entries; $report_index++ )
{
	my $timestamp   = $report_timestamps[ $report_index ];
	my @frequencies = split /,/, $report_frequencies[ $report_index ];
	my @amplitudes  = split /,/, $report_amplitudes[ $report_index ];

	my $x_dot   = $timestamp * $dot_width + $x_margin;
	my $number_of_pairs = $#frequencies;
	for( my $pair = 0; $pair < $number_of_pairs; $pair++ )
	{
		my $frequency = $frequencies[ $pair ];
		my $amplitude = $amplitudes[ $pair ];

		my $y_dot     = $frequency * $dot_height + $y_margin;
		my $x_width = $x_dot + $dot_width;
		my $y_width = $y_dot + $dot_height;

		my $heat_index = int( $amplitude * $heatmap_length );
		my $heat_name  = $color_map[ $heat_index ];

		$y_dot   = $y_size - $y_dot;
		$y_width = $y_size - $y_width;
		$coordinate_system->Draw( stroke=>$heat_name, fill=>$heat_name,
								  primitive=>"rectangle",
								  points=>"$x_dot,$y_dot $x_width,$y_width" );
	}
}
# Blur the image slightly to make it easier on the eyes.
$coordinate_system->AdaptiveBlur( radius => 3, sigma => 1.2 );

# This would be a good place to plot axes from $min_timestamp to $max_timestamp
# and $min_frequency to $max_frequency, but for now just save the image.
$coordinate_system->Write( "heatmap.png" );

