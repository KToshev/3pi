// The 3pi include file must be at the beginning of any program that
// uses the Pololu AVR library and 3pi.
#include <pololu/orangutan>
#include <pololu/Pololu3pi.h>
#include <avr/pgmspace.h>
#include <limits.h>
#include <stddef.h>
#include <stdio.h>
#include <math.h> 
#include <stdlib.h>
// Constants
const short MAX_ROWS = 15; // matrix dimensions
#define PI 3.14159265
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

void make_turn(short left_speed, short right_speed, short cnt)
{
	set_motors( left_speed, right_speed );
	delay_ms( 200 * cnt );
	
	set_motors( 0, 0 );
}

void turn_half_left(short turns)
{
	make_turn( -42, 42, turns );
}

void turn_half_right(short turns)
{
	make_turn( 41, -41, turns );
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

int GetRobotAngle(){
	switch(orientation){
		case EOrientation::South : return -90;		
		case EOrientation::SouthEast : return -45;		
		case EOrientation::East : return 0;
		case EOrientation::NorthEast: return 45;
		case EOrientation::North : return 90;
		case EOrientation::NorthWest: return 135;	
		case EOrientation::West : return 180;							
		case EOrientation::SouthWest :
		default: return 225;
	}
}

void SetDeviceOrientation(int angle){
	switch(angle){
		case -90: orientation = EOrientation::South ;break;
		case -45 : orientation = EOrientation::SouthEast;break;				
		case 0 : orientation = EOrientation::East;break;
		case 45: orientation = EOrientation::NorthEast;break;		
		case 90: orientation = EOrientation::North;break;
		case 135: orientation = EOrientation::NorthWest;break;
		case 180 : orientation = EOrientation::West  ;break;						
		case 225: orientation = EOrientation::SouthWest ;break;
	}	
}

//1,2,3,4 clockwise, -1,-2,-3,-4 anticlockwise
void SetDirection(Point2D startPoint, Point2D endPoint){
	short x = startPoint.x - endPoint.x;
	short y = startPoint.y - endPoint.y;
	int angle = 0;
	int direction = 0;
	int robotAngle = GetRobotAngle();
	if(x == 0)
	{
		if(y > 0)
		{
			angle = 90;
		}
		else if(y < 0)
		{
			angle = -90;
		}
	} 
	else{
		angle = (int)(atan(y/x)* 180 / PI);
		if(x < 0) angle += 180;
	}
	direction = (int)(round((robotAngle - angle)/45));

	if(direction > 4)
	{
		direction = -8 + direction;
	}
	if(direction < -4)
	{
		direction = 8 + direction;	
	}	
	
	SetDeviceOrientation(angle);
	if(direction < 0){
		//turn left
		set_motors( -41, 41 );
		delay_ms( abs(direction)*200 );
	}
	else if(direction > 0){
		//turn right
		set_motors( 41, -41 );
		delay_ms( abs(direction)*200 );
	}
	set_motors( 0, 0 );
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
	
	// Set Down of the robot.
	set_motors( 0, 0 );
	while ( true )
	{
		// Call this in the end in order not to let the robot execute random code!!!
	}
}