// The 3pi include file must be at the beginning of any program that uses the Pololu AVR library and 3pi.
#include <pololu/orangutan>
#include <pololu/Pololu3pi.h>
#include <avr/pgmspace.h>
#include <limits.h>
#include <stddef.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

// Constants
const short MAX_ROWS       = 15; // Matrix dimensions
const int   OBSTACLE_VALUE = 1200; // TODO: Set real value
const int   MARKER_VALUE   = 350;
#define     PI               3.14159265

// Enums
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

// Classes and global variables
class Point2D
{
    public:
        short x;
        short y;

        Point2D( short inX, short inY )
            : x( inX )
            , y ( inY )
        {}
};

class Cell
{
    public:
        short		distToStart;
        short		distToFinish;
        bool		isObstacle;
        Point2D*	parent;
        bool		isVisited;

        Cell()
            : distToStart( INT_MAX )
            , distToFinish( INT_MAX )
            , isObstacle( false )
            , isVisited( false )
        {}

        void operator = ( const Cell& other )
        {
            distToStart		= other.distToStart;
            distToFinish	= other.distToFinish;
            isObstacle		= other.isObstacle;
        }
};

Cell matrix[ MAX_ROWS ][ MAX_ROWS ];

class Robot
{
    private:
        Point2D			position;
        EOrientation	orientation;
        Point2D			finishPos;
        unsigned int	sensors[ 5 ]; // An array to hold sensor values

    public:
        Robot()
            : position( 0, 0 )
            , orientation( EOrientation::North )
            , finishPos( 0, 3 )
        {}

        void initialize()
        {
            // Initialize the line reading sensors
            pololu_3pi_init( 2000 );

            set_motors( 0, 0 );

            // Initialize the menu
            this->initializeMenu();
        }

        // Main logic function
        void mainRobotLogic()
        {
            bool isLastStep	= false;

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

        // Displays the data returned from the sensors
        void testSensors()
        {
            while ( true )
            {
                read_line_sensors( sensors, IR_EMITTERS_ON );

                // Display the sensor values
                for ( short i = 0; i < 5; i++ )
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
        void makeTurn( short leftSpeed, short rightSpeed, short turns )
        {
            set_motors( leftSpeed, rightSpeed );

            delay_ms( 200 * turns );

            set_motors( 0, 0 );
        }

        void turnHalfLeft( short turns )
        {
            makeTurn( -42, 42, turns );
        }

        void turnHalfRight( short turns )
        {
            makeTurn( 42, -42, turns );
        }

        void moveForward()
        {
            set_motors( 20, 20 );

            while ( !isOnMarker() ) {}

            play( ">>a32" );
            delay_ms( 900 );
            set_motors( 0, 0 );
        }

        bool isOnMarker()
        {
            read_line_sensors( sensors, IR_EMITTERS_ON );

            if ( sensors[1] > OBSTACLE_VALUE || sensors[2] > OBSTACLE_VALUE || sensors[3] > OBSTACLE_VALUE ) // there is obstacle
            {
                matrix[ position.x ][ position.y ].isObstacle = true;
                return true;
            }
            else if ( sensors[1] > MARKER_VALUE || sensors[2] > MARKER_VALUE || sensors[3] > MARKER_VALUE ) // there is marker
            {
                return true;
            }

            return false;
        }

        int getRobotAngle()
        {
            switch ( orientation )
            {
                case EOrientation::South:
                    return -90;

                case EOrientation::SouthEast:
                    return -45;

                case EOrientation::East:
                    return 0;

                case EOrientation::NorthEast:
                    return 45;

                case EOrientation::North:
                    return 90;

                case EOrientation::NorthWest:
                    return 135;

                case EOrientation::West:
                    return 180;

                case EOrientation::SouthWest:
                default:
                    return 225;
            }
        }

        void setDeviceOrientation( int angle )
        {
            switch ( angle )
            {
                case -90:
                    orientation = EOrientation::South;
                    break;

                case -45:
                    orientation = EOrientation::SouthEast;
                    break;

                case 0:
                    orientation = EOrientation::East;
                    break;

                case 45:
                    orientation = EOrientation::NorthEast;
                    break;

                case 90:
                    orientation = EOrientation::North;
                    break;

                case 135:
                    orientation = EOrientation::NorthWest;
                    break;

                case 180:
                    orientation = EOrientation::West;
                    break;

                case 225:
                    orientation = EOrientation::SouthWest;
                    break;
            }
        }

        //1,2,3,4 clockwise, -1,-2,-3,-4 counterclockwise
        void setDirection( Point2D startPoint, Point2D endPoint )
        {
            short	x			= -startPoint.x + endPoint.x;
            short	y			= -startPoint.y + endPoint.y;
            int		angle		= 0;
            int		direction	= 0;
            int		robotAngle	= getRobotAngle();

            if ( x == 0 )
            {
                if ( y > 0 )
                {
                    angle = 90;
                }
                else if ( y < 0 )
                {
                    angle = -90;
                }
            }
            else
            {
                angle = ( int )( atan( y / x ) * 180 / PI );

                if ( x < 0 )
                {
                    angle += 180;
                }
            }

            direction = ( int )( round( ( robotAngle - angle ) / 45 ) );

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

            setDeviceOrientation( angle );

            if ( direction < 0 )
            {
                // turn left
                turnHalfLeft( abs( direction ) );
            }
            else if ( direction > 0 )
            {
                // turn right
                turnHalfRight( abs( direction ) );
            }

            set_motors( 0, 0 );
        }

        short getSign( short num )
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

        // Returns the distance between two adjacent positions
        short getDistance( Point2D prevPos, Point2D nextPos )
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

        int childernNodesCoords [2][8] = {{1, 1, 1, 0, 0, -1, -1, -1}, {0, -1, 1, 1, -1, 0, 1, -1}};

        Point2D getNextPos( const Point2D& goalPos )
        {
            //Point2D			nextPos( position.x + getSign(goalPos.x - position.x),
            //position.y + getSign(goalPos.y - position.y) );

            Point2D nextPos( -1, -1 );

            for ( int i = 0; i < 8; i++ )
            {
                short x = position.x + childernNodesCoords[i][0];
                short y = position.y + childernNodesCoords[i][1];

                if ( x < 0 || x > MAX_ROWS || y < 0 || y > MAX_ROWS || matrix[x][y].isObstacle || matrix[x][y].isVisited )
                {
                    continue;
                }

                Point2D tempPos( x, y );

                if ( nextPos.x == -1 )
                {
                    nextPos.x = x;
                    nextPos.y = y;
                }
                else if ( heuristicDist( nextPos, goalPos ) > heuristicDist( tempPos, goalPos ) )
                {
                    nextPos = tempPos;
                }
            }

            // Get nearest possible move, sort all adjacent cells by distance and get the closest that is ok

            return nextPos;
        }

        short heuristicDist( const Point2D& start, const Point2D& end )
        {
            return abs( start.x - end.x ) + abs( start.y - end.y );
        }

        Point2D nextStepToFinish( const Point2D& goalPos )
        {
            //EOrientation	backwardDirection = GetBackwardDirection();
            Point2D	nextPos( -1, -1 );
            int		count = 0;

            while ( count < 8 )
            {
                count++;
                nextPos = this->getNextPos( goalPos );

                // There isn't an appropriate next node
                if ( nextPos.x == -1 )
                {
                    // If prevPos == NULL, we are at the beginning, print impassable matrix
                    nextPos = *matrix[ position.x ][ position.y ].parent;
                }

                setDirection( position, nextPos );
                // TODO: Check isObstacle or maybe isVisited

                if ( isDirectionTraversable() )
                {
                    return nextPos;
                }
                else
                {
                    matrix[ nextPos.x ][ nextPos.y ].isObstacle = true;
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

        Point2D getNextStepByOrientation( EOrientation orientation )
        {
            switch ( orientation )
            {
                case EOrientation::South:
                    return Point2D( position.x, position.y - 1 );

                case EOrientation::SouthEast:
                    return Point2D( position.x + 1, position.y - 1 );

                case EOrientation::East:
                    return Point2D( position.x + 1, position.y );

                case EOrientation::NorthEast:
                    return Point2D( position.x + 1, position.y + 1 );

                case EOrientation::North:
                    return Point2D( position.x, position.y + 1 );

                case EOrientation::NorthWest:
                    return Point2D( position.x - 1, position.y + 1 );

                case EOrientation::West:
                    return Point2D( position.x - 1, position.y );

                case EOrientation::SouthWest:
                    return Point2D( position.x - 1, position.y - 1 );
            }

            return position;
        }

        EOrientation getBackwardDirection()
        {
            switch ( orientation )
            {
                case EOrientation::South:
                    return EOrientation::North;

                case EOrientation::SouthEast:
                    return EOrientation::NorthWest;

                case EOrientation::East:
                    return EOrientation::West;

                case EOrientation::NorthEast:
                    return EOrientation::SouthWest;

                case EOrientation::North:
                    return EOrientation::South;

                case EOrientation::NorthWest:
                    return EOrientation::SouthEast;

                case EOrientation::West:
                    return EOrientation::East;

                case EOrientation::SouthWest:
                    return EOrientation::NorthEast;
            }
        }

        bool isDirectionTraversable()
        {
            unsigned int	sensors[ 5 ];
            read_line_sensors( sensors, IR_EMITTERS_ON );

            if ( sensors[ 1 ] > OBSTACLE_VALUE || sensors[ 2 ] > OBSTACLE_VALUE || sensors[ 3 ] > OBSTACLE_VALUE )
            {
                clear();
                print( "!OBS!" );
                delay_ms( 2000 );
                return false;
            }

            clear();
            print( "NOT OBS" );
            delay_ms( 2000 );

            return true;
        }


        void stepToGoal( Point2D goalPos )
        {
            Point2D nextPos = nextStepToFinish( goalPos );
            *matrix[nextPos.x][nextPos.y].parent = position;

            printPos( nextPos.x, nextPos.y );
            delay_ms( 2000 );
            // delay_ms(1000);

            if ( !matrix[ nextPos.x ][ nextPos.y ].isVisited )
            {
                short savedDist	= matrix[ nextPos.x ][ nextPos.y ].distToStart;
                short currDist	= matrix[ position.x ][ position.y ].distToStart + getDistance( position, nextPos );

                if ( savedDist > currDist )
                {
                    matrix[ nextPos.x ][ nextPos.y ].distToStart = currDist;
                }

                matrix[nextPos.x][nextPos.y].isVisited = true;
            }

            // Move until stepping on the goal position
            if ( position.x != goalPos.x || position.y != goalPos.y )
            {
                position = nextPos;

                moveForward();
            }

        }

        void printPos( short x, short y )
        {
            clear();
            print( "(" );
            print_long( x );
            print( ", " );
            print_long( y );
            print( ")" );
        }

        void initializeMenu()
        {
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

                printPos( finishPos.x, finishPos.y );

                lcd_goto_xy( 0, 1 );
                print( " A B C " );

                delay_ms( 100 );
            }

            // Always wait for the button to be released so that 3pi doesn't start moving until your hand is away from it.
            wait_for_button_release( BUTTON_B );
            delay_ms( 1000 );
        }

        void reverseDirection()
        {
            this->calculateDistToFinish();

            // Swap distance to start with distance to finish and find start position
            Point2D startPos( finishPos );

            for ( short i = 0; i < MAX_ROWS; i++ )
            {
                for ( short j = 0; j < MAX_ROWS; j++ )
                {
                    if ( /*matrix[ i ][ j ].isVisited*/true )
                    {
                        if ( matrix[ i ][ j ].distToStart == 0 )
                        {
                            // Save the start position.
                            startPos = Point2D( i, j );
                        }

                        short tmp						= matrix[ i ][ j ].distToFinish;
                        matrix[ i ][ j ].distToFinish	= matrix[ i ][ j ].distToStart;
                        matrix[ i ][ j ].distToStart	= tmp;
                    }
                }
            }

            // Make the start position our next finish position
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

        Point2D getNearestToStartAdjecent( const Point2D& currentPos )
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
};

int main()
{
    Robot robot;

    // Set up the 3pi
    robot.initialize();

    robot.mainRobotLogic();

    set_motors( 0, 0 );

    // Stop code execution
    while ( true ) {}
}
