// The 3pi include file must be at the beginning of any program that
// uses the Pololu AVR library and 3pi.
#include <pololu/3pi.h>
#include <avr/pgmspace.h>

// Constants
const int MAX_ROWS = 20; // matrix dimensions

// Structures and classes
enum EOrientation
{
	North,
	NorthEast,
	East,
	SouthEast,
	South,
	SouthWest,
	West,
	NorthWest
};

struct Point2D
{
	public:
		Point2D(int inX, int inY)
			: x( inX )
			, y ( inY )
			{}
	
		int x;
		int y;	
};

struct Cell
{
	public:
		Cell()
			: distToStart( INT_MAX )
			, distToFinish( INT_MAX )
			, isObstacle( 0 )
			{}
	
	// Inner fields
		int distToStart;
		int distToFinish;
		int isObstacle;
};

Cell** matrix = nullptr;

// Globals for robot current state
Point2D			position( 0, 0 );
EOrientation	orientation		= EOrientation:North;
Point2D			startPos( 0, 0 );
Point2D			finishPos( 10, 10 );


// This is the main function, where the code starts.  All C programs
// must have a main() function defined somewhere.
int main()
{
	// set up the 3pi
	initialize();
	
	test_sensors();
	//mainRobotLogic();
}


// Initializes the matrix that represents the virtual coordinate system
void init_matrix()
{
	matrix = new Cell*[ MAX_ROWS ];
	
	for ( int i = 0; i < MAX_ROWS; i++ )
	{
		matrix[ i ] = new Cell[ MAX_ROWS ];
	}
}

// Waits for Button B to be pressed for user convenience and then
// Initializes the line reading sensors
void initialize()
{
	init_matrix();
	// Display battery voltage and wait for button press
	while ( !button_is_pressed( BUTTON_B ) )
	{
		int bat = read_battery_millivolts();
		
		clear();
		print_long( bat );
		print( "mV" );
		lcd_goto_xy( 0, 1 );
		print( "Press B" );
		
		delay_ms( 100 );
	}
	
	// Always wait for the button to be released so that 3pi doesn't
	// start moving until your hand is away from it.
	wait_for_button_release( BUTTON_B );
	delay_ms( 1000 );
	
	pololu_3pi_init( 2000 );
	set_motors( 0, 0 );
}

void turn_left()
{
	set_motors( -42, 42 );
	delay_ms( 400 );
	set_motors( 0, 0 );
}

void go_forward()
{
}

void turn_right()
{
	set_motors( 41, -41 );
	delay_ms( 400 );
	set_motors( 0, 0 );
}

void turn_back()
{
	set_motors( 41, -41 );
	delay_ms( 800 );
	set_motors( 0, 0 );
}

// Tests data returned from the sensors
void test_sensors()
{
	unsigned int	sensors[ 5 ]; // an array to hold sensor values
	
	while ( 1 )
	{
		read_line_sensors( sensors, IR_EMITTERS_ON );
		
		// Display the sensor results
		for( int i = 0; i < 5; i++ )
		{
			print_long( sensors[ i ] );
			delay_ms( 500 );
			clear();
		}
		
		// Play sound to notify that the next iteration will begin.
		play( ">>a32" );
	}
}

// Our main logic function
void mainRobotLogic()
{
	unsigned int	sensors[ 5 ]; // an array to hold sensor values
	int				moving	= 1; // This flag tells us if we have to look for the marker color. If it is 1, we are on a marker and have to decide where to go.
	int				iter	= 0; // Should be replaced with posX and posY
	
	while ( 1 )
	{
		read_line_sensors( sensors, IR_EMITTERS_ON );

		if ( sensors[1] > 800 || sensors[2] > 800 || sensors[3] > 800 ) // is_on_marker(sensors)
		{
			if ( moving )
			{
				// Decide where to go next
				iter++;
				play( ">>a32" );
				moving = 0;
				
				delay_ms( 300 );
				set_motors( 0, 0 );
				
				if ( iter == 4 )
				{
					turn_right();
				}
				else if ( iter == 6 )
				{
					turn_left();
				}
			}
		}
		else
		{
			moving = 1;
		}
		
		if ( iter < 7 )
		{
			set_motors( 20, 20 );
		}
	}
}
//Ctrl+K+D
