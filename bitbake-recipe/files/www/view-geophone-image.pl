#!/usr/bin/perl
#
# CGI script that outputs a HTML snippet with a heatmap image and buttons
# for "previous" and "next."

use strict;
use warnings;

my $image_folder = "/www/pages/geophone";
my ( $image_width, $image_height ) = ( 500, 250 );


# Read all files in the specified directory that match the filename pattern
# plus optionally a timestamp.
sub read_heatmap_titles
{
	my $directory = shift;
	my $pattern   = shift;

	my $grep_pattern = '^' . $pattern . '(-[0-9]\+)?\.jpg';

	# Read all files in the directory matching the filename pattern.
	my @file_list;
	opendir( HEATMAP_DIR, $directory ) or die $!;
	while( my $filename = readdir( HEATMAP_DIR ) )
	{
		next if( ! ( $filename =~ m/$pattern(-[0-9]+)?.jpg/ ) );
		push @file_list, $filename;
	}
	closedir( HEATMAP_DIR );
	# Sort the filenames.
	my @sorted_list = sort @file_list;

	return \@sorted_list;
}



sub insert_javascript
{
	my $file_list_ref = shift;
	my @file_list     = @$file_list_ref;

	print "<script type=\"text/javascript\">\n";
	print "var heatmap_files = [ \"";
	print join( '", "', @file_list );
	print "\" ];\n";

print "var heatmap_file_number = heatmap_files.length - 1;
var heatmap_tag = document.getElementById( 'geodata_heatmap' );";


print "function load_heatmap( heatmap_number )
{
    var dirname = location.pathname.substring( 0, location.pathname.lastIndexOf( '/' ) + 1 );
    var current_url = location.protocol + '//' + location.hostname
        + ( location.port && ':' + location.port ) //+ '/'
        + dirname;
    var filename = current_url + 'geophone/' + heatmap_files[ heatmap_number ];
    heatmap_tag.src = filename;
}";


# print "var heatmap_first_tag = document.getElementById( 'geodata_first' );";
# print "heatmap_first_tag.onclick = function( )
print "function click_heatmap_first( )
{
    heatmap_file_number = 0;
    load_heatmap( heatmap_file_number );
}";


#print "var heatmap_previous_tag = document.getElementById( 'geodata_previous' );
#heatmap_previous_tag.onclick = function( )
print "function click_heatmap_previous( )
{
    heatmap_file_number = heatmap_file_number - 1;
    if( heatmap_file_number < 0 )
    {
        heatmap_file_number = 0;
    }
    load_heatmap( heatmap_file_number );
}";

#print "var heatmap_next_tag = document.getElementById( 'geodata_next' );
#heatmap_next_tag.onclick = function( )
print "function click_heatmap_next( )
{
    heatmap_file_number = heatmap_file_number + 1;
    if( heatmap_file_number >= heatmap_files.length )
    {
        heatmap_file_number = heatmap_files.length - 1;
    }
    load_heatmap( heatmap_file_number );
}";

#print "var heatmap_last_tag = document.getElementById( 'geodata_last' );
#heatmap_last_tag.onclick = function( )
print "function click_heatmap_last( )
{
    heatmap_file_number = heatmap_files.length - 1;
    load_heatmap( heatmap_file_number );
}";

print "</script>\n";
}


my $file_list_ref = read_heatmap_titles( $image_folder, "geophone" );
my @file_list     = @$file_list_ref;
my $image_filename = $file_list[ $#file_list ];

print "<!DOCTYPE html PUBLIC \"-//W3C//DTD XHTML 1.1//EN\" \"http://www.w3.org/TR/xhtml11/DTD/xhtml11.dtd\">\n";

print "<html xmlns=\"http://www.w3.org/1999/xhtml\">\n";
print "<head>";
print "<meta http-equiv=\"Content-Type\" content=\"text/html; CHARSET=UTF-8\" />\n";
print "<link href=\"style.css\" rel=\"stylesheet\" type=\"text/css\" />\n";
print "<title>Geosensor</title>\n";
print "</head>\n";
print "<body>";

print "<div id=\"geodata\">\n";

print "<div id=\"heatmap\">";
print "<img src=\"geophone/$image_filename\" id=\"geodata_heatmap\" />";
print "</div>\n";

print "<div id=\"geodata-navigation\"><ul>";
print "<li><img src=\"nav-first.png\" id=\"geodata_first\" alt=\"First\" onclick=\"click_heatmap_first();\" /></li>";
print "<li><img src=\"nav-previous.png\" id=\"geodata_previous\" alt=\"Previous\" onclick=\"click_heatmap_previous();\" /></li>";
print "<li><img src=\"nav-next.png\" id=\"geodata_next\" alt=\"Next\" onclick=\"click_heatmap_next();\" /></li>";
print "<li><img src=\"nav-last.png\" id=\"geodata_last\" alt=\"Last\" onclick=\"click_heatmap_last();\" /></li>";
print "</ul></div>\n";

# Add JavaScript for controlling the images.
insert_javascript( \@file_list );

print "</div>\n";
print "</body></html>";
