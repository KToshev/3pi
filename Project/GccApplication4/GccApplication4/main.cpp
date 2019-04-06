// The 3pi include file must be at the beginning of any program that
// uses the Pololu AVR library and 3pi.
#include <pololu/orangutan>
#include <pololu/Pololu3pi.h>
#include <avr/pgmspace.h>
#include <limits.h>
#include <stddef.h>

// Constants
const short MAX_ROWS = 15; // matrix dimensions

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
	Point2D(short inX, short inY)
	: x( inX )
	, y ( inY )
	{}
	
	short x;
	short y;
};

struct Cell
{
	public:
	Cell()
	: distToStart( INT_MAX )
	, distToFinish( INT_MAX )
	, isObstacle( false )
	{}
	
	// Inner fields
	short distToStart;
	short distToFinish;
	bool isObstacle;
};

Cell matrix[ MAX_ROWS ][ MAX_ROWS ];

// Globals for robot current state
Point2D			position( 0, 0 );
EOrientation	orientation		= EOrientation::North;
Point2D			startPos( 0, 0 );
Point2D			finishPos( 10, 10 );


// Waits for Button B to be pressed for user convenience and then
// Initializes the line reading sensors
void initialize()
{
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

void turn_half_left(short turns)
{
	set_motors( -42, 42 );
	delay_ms( turns*200 );
	set_motors( 0, 0 );
}

void turn_half_right(short turns)
{
	set_motors( 41, -41 );
	delay_ms( turns*200 );
	set_motors( 0, 0 );
}

// Tests data returned from the sensors
void test_sensors()
{
	unsigned int	sensors[ 5 ]; // an array to hold sensor values
	
	while ( true )
	{
		read_line_sensors( sensors, IR_EMITTERS_ON );
		
		// Display the sensor results
		for( short i = 0; i < 5; i++ )
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
	bool			moving	= true; // This flag tells us if we have to look for the marker color. If it is 1, we are on a marker and have to decide where to go.
	short			iter	= 0; // Should be replaced with posX and posY
	
	while ( true )
	{
		read_line_sensors( sensors, IR_EMITTERS_ON );

		if ( sensors[1] > 800 || sensors[2] > 800 || sensors[3] > 800 ) // is_on_marker(sensors)
		{
			if ( moving )
			{
				// Decide where to go next
				iter++;
				play( ">>a32" );
				moving = false;
				
				delay_ms( 500 );
				set_motors( 0, 0 );
				
				if ( iter == 4 )
				{
					turn_half_right(2);
				}
				else if ( iter == 6 )
				{
					turn_half_left(2);
				}
			}
		}
		else
		{
			moving = true;
		}
		
		if ( iter < 7 )
		{
			set_motors( 20, 20 );
		}
	}
}
//Ctrl+K+D

// This is the main function, where the code starts.  All C programs
// must have a main() function defined somewhere.
int main()
{
	// set up the 3pi
	initialize();
	
	test_sensors();
	//mainRobotLogic();
}