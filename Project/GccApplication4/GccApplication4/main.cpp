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
const int OBSTACLE_VALUE = 1200; //@TODO: SET REAL VALUE
const int MARKER_VALUE = 350;
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
	, isVisited( false )
	{}
		
	void operator = (const Cell &c ) { 
		distToStart = c.distToStart;
		distToFinish = c.distToFinish;
		isObstacle = c.isObstacle;
	}
	
	// Inner fields
	short distToStart;
	short distToFinish;
	bool isObstacle;
	Point2D* parent;
	bool isVisited;

};

Cell matrix[ MAX_ROWS ][ MAX_ROWS ];

class Robot
{
	public:
		Robot()
			: position( 0, 0 )
			, orientation( EOrientation::North )
			, finishPos( 0, 3 )
		{
		}
		
		// Waits for Button B to be pressed for user convenience and then
		// Initializes the line reading sensors
		void initialize()
		{	
			pololu_3pi_init( 2000 );
			set_motors( 0, 0 );
			
			this->doMenu();
		}
		
		// Our main logic function
		void mainRobotLogic()
		{
			bool			isLastStep	= false;
			
			while ( position.x != finishPos.x || position.y != finishPos.y || isLastStep )
			{
										
				stepToGoal( finishPos );
				
				if ( isLastStep )
				{
					isLastStep = false;
					
					this->reverseDirection();
				}
				else if ( position.x == finishPos.x && position.y == finishPos.y )
				{
					isLastStep = true;
				}
			}
		}
		
		// Tests data returned from the sensors
		void test_sensors()
		{			
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
		
	protected:
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
			make_turn( 42, -42, turns );
		}
		
		void move_forward(){
			set_motors( 20, 20 );
			
			while(!is_on_marker()){}
			
			play( ">>a32" );
			delay_ms( 900 );
			set_motors( 0, 0 );
		}
		
		bool is_on_marker()
		{
			read_line_sensors( sensors, IR_EMITTERS_ON );
			if ( sensors[1] > OBSTACLE_VALUE || sensors[2] > OBSTACLE_VALUE || sensors[3] > OBSTACLE_VALUE ) // there is obstacle
			{
				matrix[position.x][position.y].isObstacle = true;
				return true;
			} else if ( sensors[1] > MARKER_VALUE || sensors[2] > MARKER_VALUE || sensors[3] > MARKER_VALUE ) // there is marker
			{
				return true;
			}
			return false;
		}
		
		int GetRobotAngle()
		{
			switch ( orientation )
			{
				case EOrientation::South:		return -90;
				case EOrientation::SouthEast:	return -45;
				case EOrientation::East:		return 0;
				case EOrientation::NorthEast:	return 45;
				case EOrientation::North:		return 90;
				case EOrientation::NorthWest:	return 135;
				case EOrientation::West:		return 180;
				case EOrientation::SouthWest:
				default:						return 225;
			}
		}

		void SetDeviceOrientation(int angle)
		{
			switch( angle )
			{
				case -90:	orientation = EOrientation::South;		break;
				case -45:	orientation = EOrientation::SouthEast;	break;
				case 0:		orientation = EOrientation::East;		break;
				case 45:	orientation = EOrientation::NorthEast;	break;
				case 90:	orientation = EOrientation::North;		break;
				case 135:	orientation = EOrientation::NorthWest;	break;
				case 180:	orientation = EOrientation::West;		break;
				case 225:	orientation = EOrientation::SouthWest;	break;
			}
		}

		//1,2,3,4 clockwise, -1,-2,-3,-4 anticlockwise
		void SetDirection(Point2D startPoint, Point2D endPoint)
		{
			short	x			= -startPoint.x + endPoint.x;
			short	y			= -startPoint.y + endPoint.y;
			int		angle		= 0;
			int		direction	= 0;
			int		robotAngle	= GetRobotAngle();
			
			if ( x == 0 )
			{
				if ( y > 0 )
				{
					angle = 90;
				}
				else if( y < 0 )
				{
					angle = -90;
				}
			}
			else
			{
				angle = (int)(atan(y/x)* 180 / PI);
				
				if ( x < 0 )
				{
					angle += 180;
				}
			}
			
			direction = (int)(round((robotAngle - angle)/45));
			
			if ( direction > 4 )
			{
				direction = -8 + direction;
			}
			
			if ( direction < -4 )
			{
				direction = 8 + direction;
			}
			
			clear();
			print_long( direction );
			lcd_goto_xy( 0, 1 );
			print_long( angle );
			//delay_ms(1000);
			
			SetDeviceOrientation( angle );
			if ( direction < 0 )
			{
				//turn left
				turn_half_left( abs( direction ) );
			}
			else if ( direction > 0 )
			{
				//turn right
				turn_half_right( abs( direction ) );
			}
			set_motors( 0, 0 );
		}

		short getSign(short num)
		{
			short sign = 0;
			if ( num > 0 )
			{
				sign = 1;
			}
			else if ( num < 0 )
			{
				sign = -1;
			}
			return sign;
		}

		// !!!Only for one step!!!!
		short getDistance(Point2D prevPos, Point2D nextPos)
		{
			short distance = 0;
			
			if ( prevPos.x - nextPos.x != 0 && prevPos.y - nextPos.y != 0 )
			{
				distance = 3;
			}
			else if ( prevPos.x - nextPos.x != 0 || prevPos.y - nextPos.y != 0 )
			{
				distance = 2;
			}
			return distance;
		}

		int childernNodesCoords [2][8] = {{1,1,1,0,0,-1,-1,-1},{0,-1,1,1,-1,0,1,-1}};

		Point2D getNextPos(const Point2D& goalPos)
		{
			//Point2D			nextPos( position.x + getSign(goalPos.x - position.x),
			//position.y + getSign(goalPos.y - position.y) );
			
			Point2D nextPos(-1,-1);
			for(int i=0; i<8; i++)
			{
				short x = position.x + childernNodesCoords[i][0];
				short y = position.y + childernNodesCoords[i][1];
				if(x < 0 || x > MAX_ROWS || y < 0 || y > MAX_ROWS || matrix[x][y].isObstacle || matrix[x][y].isVisited) continue;
				
				Point2D tempPos(x,y);
				if(nextPos.x == -1)
				{
					nextPos.x = x;
					nextPos.y = y;	
				}
				else if(HeuristicDist(nextPos,goalPos) > HeuristicDist(tempPos,goalPos))
				{
					nextPos = tempPos;
				}
			}
			// get nearest possible move, sort all adjacent cells by distance and get the closest that is ok!
			
			return nextPos;
		}
		//moje da promenim izchislenieto
		short HeuristicDist(const Point2D& start, const Point2D& end)
		{
			return abs(start.x - end.x) + abs(start.y - end.y);
		}

		Point2D nextStepToFinish(const Point2D& goalPos)
		{
			//EOrientation	backwardDirection = GetBackwardDirection();
			Point2D nextPos(-1,-1);
			int count = 0;
			while(count < 8)
			{
				count ++;
				nextPos = this->getNextPos( goalPos );
			
				//appropriate next node not found
				if(nextPos.x == -1)
				{
					//if prevPos == NULL, we are at beginning print impassable matrix
					nextPos = *matrix[position.x][position.y].parent;
				}
			
				SetDirection( position, nextPos );				
				//TODO: Check is obstacle or maybe is visited
			
				if ( CheckIfDirectionIsTraversable() )
				{
					return nextPos;
				}
				else
				{
					matrix[nextPos.x][nextPos.y].isObstacle = true;	
				}
			}
			
			return nextPos;
			// We are on an obstacle, look for a way around!
			//EOrientation startOrientation = orientation;
			//
			//do
			//{
				////45 degrees
				//turn_half_right(1);
				//
				//int angle = GetRobotAngle() - 45; 
				//
				//if ( angle < -90 )
				//{
					//angle = 225;
				//}
				//SetDeviceOrientation( angle );
			//}
			//while( orientation != startOrientation && ( !CheckIfDirectionIsTraversable() || backwardDirection == orientation ) );
			//
			//EOrientation finalOrientation = backwardDirection;
			//if ( startOrientation != orientation )
			//{
				//finalOrientation = orientation;
			//}
			//
			//nextPos = GetNextStepByOrientation( finalOrientation );
			//SetDirection( position, nextPos );
			//return nextPos;
		}
		
		Point2D GetNextStepByOrientation(EOrientation orientation)
		{
			switch ( orientation )
			{
				case EOrientation::South:		return Point2D(position.x, position.y-1);
				case EOrientation::SouthEast:	return Point2D(position.x+1, position.y-1);
				case EOrientation::East:		return Point2D(position.x+1, position.y);
				case EOrientation::NorthEast:	return Point2D(position.x+1, position.y+1);
				case EOrientation::North:		return Point2D(position.x, position.y+1);
				case EOrientation::NorthWest:	return Point2D(position.x-1, position.y+1);
				case EOrientation::West:		return Point2D(position.x-1, position.y);
				case EOrientation::SouthWest:	return Point2D(position.x-1, position.y-1);
			}			
			
			return position;
		}
		
		EOrientation GetBackwardDirection()
		{
			switch ( orientation )
			{
				case EOrientation::South:		return EOrientation::North;
				case EOrientation::SouthEast:	return EOrientation::NorthWest;
				case EOrientation::East:		return EOrientation::West;
				case EOrientation::NorthEast:	return EOrientation::SouthWest;
				case EOrientation::North:		return EOrientation::South;
				case EOrientation::NorthWest:	return EOrientation::SouthEast;
				case EOrientation::West:		return EOrientation::East;
				case EOrientation::SouthWest:	return EOrientation::NorthEast;
			}
		}
		
		bool CheckIfDirectionIsTraversable()
		{
			unsigned int	sensors[ 5 ]; 
			read_line_sensors( sensors, IR_EMITTERS_ON );
			
			if ( sensors[ 1 ] > OBSTACLE_VALUE || sensors[ 2 ] > OBSTACLE_VALUE || sensors[ 3 ] > OBSTACLE_VALUE )
			{
				clear();
				print("!OBS!");
				delay_ms(2000);
				return false;
			}
			
			clear();
			print("NOT OBS");
			delay_ms(2000);
			
			return true;
		}

		
		void stepToGoal(Point2D goalPos)
		{
			Point2D nextPos = nextStepToFinish( goalPos );
			*matrix[nextPos.x][nextPos.y].parent = position;
			matrix[nextPos.x][nextPos.y].isVisited = true;
			printPos( nextPos.x, nextPos.y );
			delay_ms(2000);
			// delay_ms(1000);
			
			//Set position in matrix only if it is NULL
			if ( !matrix[ nextPos.x ][ nextPos.y ].isVisited)
			{ // @TODO: How to check is it visited
				matrix[nextPos.x][nextPos.y].distToStart = matrix[position.x][position.y].distToStart +
					getDistance(position, nextPos);
				matrix[nextPos.x][nextPos.y].isVisited = true;
			}
						
			//Move only when are not at goal
			if ( position.x != goalPos.x || position.y != goalPos.y )
			{

				position = nextPos;
				move_forward();
			}
			
		}
		
		void printPos(short x, short y){
			clear();
			print("(");
			print_long( x );
			print(", ");
			print_long( y );
			print(")");
		}
		
		void doMenu()
		{
			// Display battery voltage and wait for button press
			while ( !button_is_pressed( BUTTON_B ) )
			{
				if ( button_is_pressed( BUTTON_A ) )
				{
					finishPos.x = ( finishPos.x + 1 ) % MAX_ROWS;
				}
				
				if ( button_is_pressed( BUTTON_C ) )
				{
					finishPos.y = ( finishPos.y + 1 ) % MAX_ROWS;
				}
				
				printPos(finishPos.x, finishPos.y);
				
				lcd_goto_xy( 0, 1 );
				print( " A B C " );
				
				delay_ms( 100 );
			}
			
			// Always wait for the button to be released so that 3pi doesn't
			// start moving until your hand is away from it.
			wait_for_button_release( BUTTON_B );
			delay_ms( 1000 );
		}
		
		void reverseDirection()
		{
			this->calculateDistToFinish();
			
			// swap dist to start with dist to finish and find start pos
			Point2D startPos( finishPos );
			
			for ( short i = 0; i < MAX_ROWS; i++ )
			{
				for ( short j = 0; j < MAX_ROWS; j++ )
				{	
					if ( /*matrix[ i ][ j ].isVisited*/true )
					{
						if ( matrix[ i ][ j ].distToStart == 0 )
						{
							// start pos, save it.
							startPos = Point2D( i, j );
						}
						
						short tmp						= matrix[ i ][ j ].distToFinish;
						matrix[ i ][ j ].distToFinish	= matrix[ i ][ j ].distToStart;
						matrix[ i ][ j ].distToStart	= tmp;
					}
				}
			}
			
			// Finally, make the start pos our next finish pos
			finishPos = startPos;
		}
		
		void calculateDistToFinish()
		{
			Point2D currentPos( finishPos );
			short	dist = matrix[ finishPos.x ][ finishPos.y ].distToStart;
			
			while ( matrix[ currentPos.x ][ currentPos.y ].distToStart != 0  )
			{
				Cell& currCell( matrix[ currentPos.x ][ currentPos.y ] );
				
				if ( currCell.distToFinish < dist - currCell.distToStart )
				{
					currCell.distToFinish = dist - currCell.distToStart;
				}
				
				currentPos = this->getNearestToStartAdjecent( currentPos );
			}
			
			Cell& currCell( matrix[ currentPos.x ][ currentPos.y ] );
			
			if ( currCell.distToFinish < dist - currCell.distToStart )
			{
				currCell.distToFinish = dist - currCell.distToStart;
			}
		}
		
		Point2D getNearestToStartAdjecent(const Point2D& currentPos)
		{
			Point2D nearestPos( currentPos );
			short	minDist = matrix[ currentPos.x ][ currentPos.y ].distToStart;
			
			if ( matrix[ currentPos.x - 1 ][ currentPos.y - 1 ].distToStart < minDist )
			{
				nearestPos	= Point2D( currentPos.x - 1, currentPos.y - 1 );
				minDist		= matrix[ nearestPos.x ][ nearestPos.y ].distToStart;
			}
			
			if ( matrix[ currentPos.x ][ currentPos.y - 1 ].distToStart < minDist )
			{
				nearestPos	= Point2D( currentPos.x, currentPos.y - 1 );
				minDist		= matrix[ nearestPos.x ][ nearestPos.y ].distToStart;
			}
			
			if ( matrix[ currentPos.x + 1 ][ currentPos.y - 1 ].distToStart < minDist )
			{
				nearestPos	= Point2D( currentPos.x + 1, currentPos.y - 1 );
				minDist		= matrix[ nearestPos.x ][ nearestPos.y ].distToStart;
			}
			
			if ( matrix[ currentPos.x - 1 ][ currentPos.y ].distToStart < minDist )
			{
				nearestPos	= Point2D( currentPos.x - 1, currentPos.y );
				minDist		= matrix[ nearestPos.x ][ nearestPos.y ].distToStart;
			}
			
			if ( matrix[ currentPos.x + 1 ][ currentPos.y ].distToStart < minDist )
			{
				nearestPos	= Point2D( currentPos.x + 1, currentPos.y );
				minDist		= matrix[ nearestPos.x ][ nearestPos.y ].distToStart;
			}
			
			if ( matrix[ currentPos.x - 1 ][ currentPos.y + 1 ].distToStart < minDist )
			{
				nearestPos	= Point2D( currentPos.x - 1, currentPos.y + 1 );
				minDist		= matrix[ nearestPos.x ][ nearestPos.y ].distToStart;
			}
			
			if ( matrix[ currentPos.x ][ currentPos.y + 1 ].distToStart < minDist )
			{
				nearestPos	= Point2D( currentPos.x, currentPos.y + 1 );
				minDist		= matrix[ nearestPos.x ][ nearestPos.y ].distToStart;
			}
			
			if ( matrix[ currentPos.x + 1 ][ currentPos.y + 1 ].distToStart < minDist )
			{
				nearestPos	= Point2D( currentPos.x + 1, currentPos.y + 1 );
				minDist		= matrix[ nearestPos.x ][ nearestPos.y ].distToStart;
			}
			
			return nearestPos;
		}
		
	private:
		Point2D			position;
		EOrientation	orientation;
		Point2D			finishPos;
		unsigned int	sensors[ 5 ]; // an array to hold sensor values
		
};

// This is the main function, where the code starts.  All C programs
// must have a main() function defined somewhere.
int main()
{
	Robot robot;
	// set up the 3pi
	robot.initialize();
	
	robot.mainRobotLogic();
	//robot.test_sensors();
	// Set Down of the robot.
	set_motors( 0, 0 );
	while ( true )
	{
		// Call this in the end in order not to let the robot execute random code!!!
	}
}
