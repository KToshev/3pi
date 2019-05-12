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
const short MAX_ROWS		= 15; // Matrix dimensions
const short OBSTACLE_VALUE	= 1200; // TODO: Set real value
const short MARKER_VALUE	= 350;
const short LAPS_COUNT		= 4;
const short MAX_OFFSET		= 250;
#define     PI                3.14159265

bool doPrint = false;

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

        Point2D()
        {

        }

        Point2D( short inX, short inY )
            : x( inX )
            , y ( inY )
        {}

        bool operator==( const Point2D& other ) const
        {
            return x == other.x && y == other.y;
        }

        bool operator!=( const Point2D& other ) const
        {
            return x != other.x || y != other.y;
        }
};
//oh, it's magic
short adjacentSquaresCoordinatesIteration [2][8] = {{0, 1, 1, 1, 0, -1, -1, -1}, {1, 1, 0, -1, -1, -1, 0, 1}};

class Cell
{
    public:
        short		distToStart;
        short		distToFinish;
        bool		isObstacle;
        bool		isVisited;

        Cell()
            : distToStart( SHRT_MAX )
            , distToFinish( SHRT_MAX )
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

typedef short ( *getDistFunc )( const Point2D&, const Point2D& );

Cell matrix[ MAX_ROWS ][ MAX_ROWS ];

class Robot
{
    private:
        Point2D			position;
        EOrientation	orientation;
        Point2D			finishPos;
        unsigned int	sensors[ 5 ]; // An array to hold sensor values
        short			lap;
        bool			boostLeftWheel;
        bool			boostRightWheel;

    public:
        Robot()
            : position( 5, 0 )
            , orientation( EOrientation::North )
            , finishPos( 5, 3 )
            , lap ( 0 )
            , boostLeftWheel( false )
            , boostRightWheel( false )
        {}
        void initialize()
        {
            // Initialize the line reading sensors
            pololu_3pi_init( 2000 );

            set_motors( 0, 0 );

            // Display battery voltage and wait two seconds
            unsigned short bat = read_battery_millivolts();

            clear();
            print_long( bat );
            print( "mV" );
            delay_ms( 1000 );

            this->mainMenu();
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
            if ( boostLeftWheel && !boostRightWheel )
            {
                set_motors( 20, 19 );
            }
            else if ( !boostLeftWheel && boostRightWheel )
            {
                set_motors( 19, 20 );
            }
            else
            {
                set_motors( 19, 19 );
            }

            while ( !isOnMarker() ) {}

            play( ">>a32" );
            delay_ms( 900 );
            set_motors( 0, 0 );
        }

        bool isOnMarker()
        {
            read_line_sensors( sensors, IR_EMITTERS_ON );

            short currentOffset = sensors[0] + sensors[1] - sensors[3] - sensors[4];

            if ( abs( currentOffset ) >= MAX_OFFSET )
            {
                if ( currentOffset > 0 )
                {
                    boostLeftWheel = true;
                    boostRightWheel = false;
                }
                else
                {
                    boostLeftWheel = false;
                    boostRightWheel = true;
                }
            }
            else
            {
                boostLeftWheel = false;
                boostRightWheel = false;
            }

            if ( sensors[1] > OBSTACLE_VALUE || sensors[2] > OBSTACLE_VALUE || sensors[3] > OBSTACLE_VALUE ) // there is obstacle
            {
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

        Point2D getClosestAdjacent( const Point2D& from, const Point2D& to, getDistFunc getDist )
        {
            short	minDist		= getDist( from, to );
            Point2D	minPos( from );
            short	pos			= ( short )orientation;

            for ( short i = 0; i < 8; i++ )
            {
                Point2D currPos;

                currPos.x = from.x + adjacentSquaresCoordinatesIteration[ 0 ][ ( pos + i ) % 8 ];
                currPos.y = from.y + adjacentSquaresCoordinatesIteration[ 1 ][ ( pos + i ) % 8 ];

                short	currDist = getDist( currPos, to );

                if ( doPrint )
                {
                    printPos( currPos );
                    lcd_goto_xy( 0, 1 );
                    print_long( currDist );
                    print( " | " );
                    print_long( minDist );
                    delay_ms( 4000 );
                }

                if ( currDist < minDist )
                {
                    minDist = currDist;
                    minPos	= currPos;
                }
            }

            doPrint = false;

            return minPos;
        }

        Point2D getNextPos( const Point2D& goalPos )
        {
            // Define dist lambda
            auto getNextDist = []( const Point2D & pos, const Point2D & goal ) -> short
            {
                short result = SHRT_MAX;

                if ( Robot::isValidPos( pos ) &&
                        !matrix[ pos.x ][ pos.y ].isObstacle && !matrix[ pos.x ][ pos.y ].isVisited )
                {
                    result = abs( goal.x - pos.x ) + abs( goal.y - pos.y );
                }

                return result;
            };

            // Get next pos using the lambda from above
            Point2D nextPos( this->getClosestAdjacent( position, goalPos, getNextDist ) );

            if ( nextPos == position )
            {

                clear();
                print( "here" );
                delay_ms( 1000 );
                clear();

                // Could not find next pos with getNextDist, try with getDistToStart
                // Define dist lambda
                auto getDistToStart = []( const Point2D & pos, const Point2D & tmp ) -> short
                {
                    short result = SHRT_MAX;

                    if ( Robot::isValidPos( pos ) )
                    {
                        result = matrix[ pos.x ][ pos.y ].distToStart;
                    }

                    return result;
                };

                // Get next pos using the lambda from above
                nextPos = this->getClosestAdjacent( position, position, getDistToStart );
            }

            return nextPos;
        }

        Point2D nextStepToFinish( const Point2D& goalPos )
        {
            //EOrientation	backwardDirection = GetBackwardDirection();
            Point2D	nextPos( -1, -1 );
            int		count = 0;

            while ( count < 8 )
            {
                count++;

                if ( lap < LAPS_COUNT )
                {
                    nextPos = this->getNextPos( goalPos );
                }
                else
                {
                    nextPos = this->getNearestToFinishAdjacent( position );
                }

                if ( !( position == nextPos ) )
                {
                    setDirection( position, nextPos );
                }

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
        }

        Point2D getNextStepByOrientation( EOrientation inOrientation )
        {
            switch ( inOrientation )
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

        bool isDirectionTraversable()
        {
            unsigned int	sensors[ 5 ];
            read_line_sensors( sensors, IR_EMITTERS_ON );

            if ( sensors[ 1 ] > OBSTACLE_VALUE || sensors[ 2 ] > OBSTACLE_VALUE || sensors[ 3 ] > OBSTACLE_VALUE )
            {
                clear();
                print( "!OBS!" );
                delay_ms( 1000 );

                return false;
            }

            clear();
            print( "NOT OBS" );
            delay_ms( 1000 );

            return true;
        }


        void stepToGoal( const Point2D& goalPos )
        {
            Point2D nextPos = nextStepToFinish( goalPos );

            printPos( nextPos );
            delay_ms( 1000 );

            if ( !matrix[ nextPos.x ][ nextPos.y ].isVisited )
            {
                // Update distToStart of nextPos based on the distToStart to the closest to the start adj pos.
                Point2D	closestToStartAdj( this->getNearestToStartAdjacent( nextPos ) );
                short	savedDist	= matrix[ nextPos.x ][ nextPos.y ].distToStart;
                short	currDist	= matrix[ closestToStartAdj.x ][ closestToStartAdj.y ].distToStart + getDistance( closestToStartAdj, nextPos );

                if ( savedDist > currDist )
                {
                    matrix[ nextPos.x ][ nextPos.y ].distToStart = currDist;
                }

                matrix[nextPos.x][nextPos.y].isVisited = true;
            }

            // Move until stepping on the goal position
            if ( position != goalPos )
            {
                position = nextPos;

                moveForward();
            }

        }

        // Main logic function
        void mainRobotLogic()
        {
            matrix[ position.x ][ position.y ].distToStart	= 0;
            matrix[ position.x ][ position.y ].isVisited	= true;

            while ( ( position != finishPos ) && lap <= LAPS_COUNT )
            {
                stepToGoal( finishPos );

                if ( position == finishPos )
                {
                    matrix[ position.x ][ position.y ].distToFinish	= 0;
                    this->reverseDirection();
                    doPrint = true;
                    matrix[ position.x ][ position.y ].isVisited	= true;

                    lap++;
                }
            }
        }

        void printPos( const Point2D& pos )
        {
            clear();
            print( "(" );
            print_long( pos.x );
            print( ", " );
            print_long( pos.y );
            print( ")" );
        }

        void initializeMenu()
        {
            clear();

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

                printPos( finishPos );

                lcd_goto_xy( 0, 1 );
                print( " A B C " );

                delay_ms( 100 );
            }

            // Always wait for the button to be released so that 3pi doesn't start moving until your hand is away from it.
            wait_for_button_release( BUTTON_B );
            delay_ms( 1000 );
        }

        //Main program menu
        void mainMenu()
        {
            clear();
            print( "A - Test" );
            lcd_goto_xy( 0, 1 );
            print( "B - Main" );

            bool isSelectedProgram = false;

            while ( !isSelectedProgram )
            {
                if ( button_is_pressed( BUTTON_A ) )
                {
                    wait_for_button_release( BUTTON_A ); //wait for the button to be released before run the program
                    testSensors();
                }

                if ( button_is_pressed( BUTTON_B ) )
                {
                    wait_for_button_release( BUTTON_B );//wait for the button to be released before run the program
                    // mainRobotLogic the initialization menu
                    initializeMenu();
                    mainRobotLogic();
                }
            }
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
                    if ( matrix[ i ][ j ].isVisited )
                    {
                        if ( matrix[ i ][ j ].distToStart == 0 )
                        {
                            // Save the start position.
                            startPos = Point2D( i, j );
                        }

                        short tmp						= matrix[ i ][ j ].distToFinish;
                        matrix[ i ][ j ].distToFinish	= matrix[ i ][ j ].distToStart;
                        matrix[ i ][ j ].distToStart	= tmp;
                        matrix[ i ][ j ].isVisited		= false;
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

                currentPos = this->getNearestToStartAdjacent( currentPos );
            }

            Cell& currCell( matrix[ currentPos.x ][ currentPos.y ] );

            if ( currCell.distToFinish < dist - currCell.distToStart )
            {
                currCell.distToFinish = dist - currCell.distToStart;
            }
        }

        Point2D getNearestToStartAdjacent( const Point2D& currentPos )
        {
            auto getDistToStart = []( const Point2D & pos, const Point2D & tmp ) -> short
            {
                short result = SHRT_MAX;

                if ( Robot::isValidPos( pos ) && matrix[ pos.x ][ pos.y ].isVisited )
                {
                    result = matrix[ pos.x ][ pos.y ].distToStart;
                }

                return result;
            };

            return this->getClosestAdjacent( currentPos, currentPos, getDistToStart );
        }

        Point2D getNearestToFinishAdjacent( const Point2D& currentPos )
        {
            auto getDistToFinish = []( const Point2D & pos, const Point2D & tmp ) -> short
            {
                short result = SHRT_MAX;

                if ( Robot::isValidPos( pos ) && !matrix[ pos.x ][ pos.y ].isVisited )
                {
                    result = matrix[ pos.x ][ pos.y ].distToFinish;
                }

                return result;
            };

            return this->getClosestAdjacent( currentPos, currentPos, getDistToFinish );
        }

        static bool isValidPos( const Point2D& pos )
        {
            return pos.x >= 0 && pos.x < MAX_ROWS && pos.y >= 0 && pos.y < MAX_ROWS;
        }
};

int main()
{
    Robot robot;

    // Set up the 3pi
    robot.initialize();

    set_motors( 0, 0 );

    // Stop code execution
    while ( true ) {}
}
