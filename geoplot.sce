// Scilab script that loads a matrix with the geophone log file and plots
// it in a heatmap.

geophone_log = mopen( "geophone.log", "rt" );
log_entries = mgetl( geophone_log );
num_entries = size( log_entry );
mclose( geophone_log );

for x_time = 1:num_entries(1)

	entry = log_entries( x_time );

	y_tokens = [];
	z_tokens = [];
	log_token = strtok( entry, "," );
	y_tokens = [ y_tokens, log_token ];
	while( log_token <> '' )
		log_token = strtok( "," );
		z_tokens = [ z_tokens, log_token ];
		log_token = strtok( "," );
		y_tokens = [ y_tokens, log_token ];
	end

	y_elements=size(y_tokens);
	y_tokens=y_tokens(1:y_elements(2)-1);
	y_elements = size( y_tokens );
	for elements = 1:y_elements(2)
		frequency = strtod( y_tokens(elements) );
		amplitude = strtod( z_tokens(elements) );
		z_amplitude(x_time,frequency) = amplitude;
	end

end

z_size  = size(z_amplitude);
maxfreq = z_size(2);
y_frequency = 1:maxfreq;
x_time = 1:x_time;

f = scf( );
plot3d1( x_time, y_frequency, z_amplitude, alpha=60.5, theta=-90 );
f.color_map = hotcolormap(32);
