#!/usr/bin/perl
#
# Create a heatmap based on the frequencies and amplitudes versus
# time read from a log file specified as argument.

use strict;
use Image::Magick;
use Date::Parse;
use Data::Dumper qw(Dumper);

my $logfile = shift;
if( $logfile eq "" )
{
	print "USAGE:  createheatmap.pl <logfile>\n";
	exit;
}

# Set the desired colormap among those listed in %color_maps.
my $desired_colormap = "spectral";

# The total number of pixels in the Y direction.
my $y_size       = 220;
# Margins for the frequency plot, not including axes.
my $x_margin_l   = 40;
my $x_margin_r   = 40;
my $y_margin_t   = 5;
my $y_margin_b   = 36;
# Set tick distances.
my $x_resolution = 60; # Seconds
my $y_resolution = 50; # Hz
# Date format for x axis.
my $datestring = "%H:%M:%S";

my $font = "/usr/share/fonts/truetype/msttcorefonts/arial.ttf";

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
open( LOG, $logfile ) || die;
my @log = <LOG>;
close( LOG );

# Read the log, assuming for now that it doesn't contain errors.
# Create a report which is an array of triplets:  a timestamp, an array of
# frequencies, and an array of their corresponding amplitudes.
my @report_timestamps;
my @report_frequencies;
my @report_amplitudes;
my $report_index = 0;
my $timestamp = 22;
my $min_timestamp = 1.0e12;
my $max_timestamp = 0;
my $min_frequency = 0;
my $max_frequency = 0;
my $has_real_timestamps = 0;
foreach my $log_entry( @log )
{
	my @frequencies;
	my @amplitudes;

	my $log_entry_begin = substr $log_entry, 0, 2;
	if( $log_entry_begin != "E:" && $log_entry_begin != "OK" )
	{
		# If the first log entry contains a timestamp then extract it.
		# Otherwise, assume there's an entry per second (beginning at
		# time = 0).
		my $test = $log_entry;
		$test =~ /([a-zA-Z]{3} [a-zA-Z]{3} [0-9]{1,2} [0-9]{2}:[0-9]{2}:[0-9]{2} [a-zA-Z]{3} [0-9]{4}) (.*)/;
		my( $log_timestamp, $measurements ) = ( $1, $2 );
		if( $log_timestamp eq "" )
		{
			$log_timestamp = $timestamp;
			$measurements  = $log_entry;
		}
		else
		{
			# Convert the text timestamp to UNIX time.
			$has_real_timestamps = 1;
			my $unix_time = str2time( $log_timestamp );
			$unix_time = $unix_time->epoch;
			print "T: $unix_time\n";
		}
		$report_timestamps[ $report_index ] = $log_timestamp;
		if( $log_timestamp < $min_timestamp )
		{
			$min_timestamp = $log_timestamp;
		}
		if( $log_timestamp > $max_timestamp )
		{
			$max_timestamp = $log_timestamp;
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

# Create an empty canvas.
my $coordinate_system = Image::Magick->new( );
my $dot_width  = 1;
if( $#report_timestamps < 200 )
{
	$dot_width = 4;
}
my $plot_width = $x_margin_l + $x_margin_r
	+ ( $max_timestamp - $min_timestamp + 1 ) * $dot_width;
$coordinate_system->Set( size => "$plot_width" . "x$y_size" );
$coordinate_system->ReadImage( "canvas:black" );
# Compute the dot height based on the height of the image.
my $number_of_frequency_components = 256;
#my $dot_height = int( ( $y_size - 2 * $y_margin ) / $number_of_frequency_components );
my $dot_height = int( ( $y_size - $y_margin_t - $y_margin_b ) / $max_frequency );
my $x_size = $coordinate_system->Get( "width" );

# Find the x translation and scaling.
my $coord_width_x = $x_size - $x_margin_r - $x_margin_l;
my $first_x_aligned_timestamp = int( $min_timestamp / $x_resolution ) * $x_resolution;
my $last_x_aligned_timestamp  = int( ( $max_timestamp + $x_resolution ) / $x_resolution ) * $x_resolution;


# Translate and scale according to the physical width of the coordinate
# system.
sub translate_x
{
	my $new_x = shift;

	$new_x = $new_x - $first_x_aligned_timestamp;
	$new_x = $new_x * $coord_width_x / ( $last_x_aligned_timestamp - $first_x_aligned_timestamp ) + $x_margin_l;
	return $new_x;
}


# Plot the dots, colored according to the color map.
my $entries = $#report_timestamps;
for( my $report_index = 0; $report_index < $entries; $report_index++ )
{
	my $timestamp   = $report_timestamps[ $report_index ];
	my @frequencies = split /,/, $report_frequencies[ $report_index ];
	my @amplitudes  = split /,/, $report_amplitudes[ $report_index ];

	# Translate the x position according to the lowest timestamp.
	my $x_dot = translate_x( $timestamp );

	my $number_of_pairs = $#frequencies;
	for( my $pair = 0; $pair < $number_of_pairs; $pair++ )
	{
		my $frequency = $frequencies[ $pair ];
		my $amplitude = $amplitudes[ $pair ];

		my $y_dot     = $frequency * $dot_height + $y_margin_b;
		my $x_width = $x_dot + $dot_width;
		my $y_width = $y_dot + $dot_height;
		$y_dot   = $y_size - $y_dot;
		$y_width = $y_size - $y_width;
		my $heat_index = int( $amplitude * $heatmap_length );
		my $heat_name  = $color_map[ $heat_index ];

		$coordinate_system->Draw( stroke => $heat_name, fill => $heat_name,
								  primitive => "rectangle",
								  points => "$x_dot,$y_dot $x_width,$y_width" );
	}
}
# Blur the image slightly to make it easier on the eyes.
$coordinate_system->AdaptiveBlur( radius => 3, sigma => 1.2 );

# Plot axes.
my $coord_origin_x = $x_margin_l;
my $coord_origin_y = $y_margin_b;
my $coord_end_x    = translate_x( $last_x_aligned_timestamp );
my $coord_end_y    = $max_frequency * $dot_height + $y_margin_b;
$coord_origin_y = $y_size - $coord_origin_y;
$coord_end_y    = $y_size - $coord_end_y;
my $line_from_x = $coord_end_x + 4; my $line_from_y = $coord_origin_y + 0;
my $line_to_x = $coord_origin_x - 1; my $line_to_y = $coord_origin_y + 0;
$coordinate_system->Draw( stroke => "white", primitive => "line",
						  points => "$line_from_x,$line_from_y "
						  . "$line_to_x,$line_to_y" );
$line_from_x = $coord_origin_x - 1; $line_from_y = $coord_end_y - 3;
$line_to_x = $coord_origin_x - 1; $line_to_y = $coord_origin_y + 0;
$coordinate_system->Draw( stroke => "white", primitive => "line",
						  points => "$line_from_x,$line_from_y "
						  . "$line_to_x,$line_to_y" );

# Plot tick marks and values on the axes.
my $x_value = $first_x_aligned_timestamp;
for( my $x_tick_pos = $first_x_aligned_timestamp;
	 $x_tick_pos <= $max_timestamp + $x_resolution;
	 $x_tick_pos += $x_resolution )
{
	# Translate the tick position by the lowest time stamp.
	my $x_tick_pos_l = translate_x( $x_tick_pos );

	# Plot the tick.
	if( $x_value != $first_x_aligned_timestamp )
	{
		$line_from_x = $x_tick_pos_l; $line_from_y = $coord_origin_y + 1;
		$line_to_x = $x_tick_pos_l; $line_to_y = $coord_origin_y + 5;
		$coordinate_system->Draw( stroke => "white", primitive => "line",
								  points => "$line_from_x,$line_from_y "
								  . "$line_to_x,$line_to_y" );
	}
	# Write the tick value.
	my $x_value_x = $x_tick_pos_l;
	my $x_value_y = $coord_origin_y + 18;
	if( $has_real_timestamps == 0 )
	{
		$coordinate_system->Annotate( font => "$font", text => "$x_value",
									  pointsize => "9", antialias => "true",
									  align => "center", fill => "white",
									  x => "$x_value_x", y => "$x_value_y" );
	}
	# Write the time stamps as date strings.
	else
	{
		my $x_date = localtime( $x_value)->strftime( "$datestring" );
		$coordinate_system->Annotate( font => "$font", text => "$x_date",
									  pointsize => "9", antialias => "true",
									  align => "center", fill => "white",
									  x => "$x_value_x", y => "$x_value_y",
									  rotate => "-90" );
	}
	$x_value = $x_value + $x_resolution;

}
my $y_value = 0;
for( my $y_tick_pos_b = $coord_origin_y; $y_tick_pos_b >= $coord_end_y;
	 $y_tick_pos_b -= $y_resolution * $dot_height )
{
	# Plot the tick.
	if( $y_value != 0 )
	{
		$line_from_x = $coord_origin_x - 5; $line_from_y = $y_tick_pos_b;
		$line_to_x = $coord_origin_x - 1; $line_to_y = $y_tick_pos_b;
		$coordinate_system->Draw( stroke => "white", primitive => "line",
								  points => "$line_from_x,$line_from_y "
								  . "$line_to_x,$line_to_y" );
	}
	# Write the tick value.
	my $y_value_x = 32;
	my $y_value_y = $y_tick_pos_b + 4;
	$coordinate_system->Annotate( font => "$font", text => "$y_value",
								  pointsize => "9", antialias => "true",
								  align => "right", fill => "white",
								  x => "$y_value_x", y => "$y_value_y" );
	$y_value = $y_value + $y_resolution;
}

# Plot labels.
my $x_label_x = ( $coord_end_x + $coord_origin_x ) / 2;
my $x_label_y = $y_size - 5;
$coordinate_system->Annotate( font => "$font", text => "Time",
							  pointsize => "13", antialias => "true",
							  align => "center", fill => "white",
							  x => "$x_label_x", y => "$x_label_y" );
my $y_label_x = 12;
my $y_label_y = ( $coord_origin_y + $coord_end_y ) / 2;
$coordinate_system->Annotate( font => "$font", text => "Frequency",
							  pointsize => "13", antialias => "true",
							  align => "center", fill => "white",
							  x => "$y_label_x", y => "$y_label_y",
							  rotate => "-90" );

# Write the heapmap image to disk.
$coordinate_system->Write( filename => "heatmap.jpg", compression => "JPEG" );
