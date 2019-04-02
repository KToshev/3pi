// The 3pi include file must be at the beginning of any program that
// uses the Pololu AVR library and 3pi.

#include <pololu/3pi.h>
//#include <avr/pgmspace.h>
//#include <stdlib.h>
//#include <stdbool.h>
//#include <pololu/3pi.h>

// This include file allows data to be stored in program space.  The
// ATmegaxx8 has 16x more program space than RAM, so large
// pieces of static data should be stored in program space.
#include <avr/pgmspace.h>

// Introductory messages.  The "PROGMEM" identifier causes the data to
// go into program space.
const char welcome_line2[] PROGMEM = "3\xf7 Robot";
const char demo_name_line1[] PROGMEM = "Line";
const char demo_name_line2[] PROGMEM = "follower";

// A couple of simple tunes, stored in program space.
const char welcome[] PROGMEM = ">g32>>c32";
const char go[] PROGMEM = "L16 cdegreg4";


// Initializes the 3pi, displays a welcome message, calibrates, and
// plays the initial music.
void initialize()
{
	unsigned int counter; // used as a simple timer
	//unsigned int sensors[5]; // an array to hold sensor values
	
	// This must be called at the beginning of 3pi code, to set up the
	// sensors.  We use a value of 2000 for the timeout, which
	// corresponds to 2000*0.4 us = 0.8 ms on our 20 MHz processor.
	delay_ms(2000);
	
	pololu_3pi_init(2000);
	
	// Auto-calibration: turn right and left while calibrating the
	// sensors.
	/*for(counter=0;counter<80;counter++)
	{
		if(counter < 20 || counter >= 60)
		set_motors(40,-40);
		else
		set_motors(-40,40);
		
		// This function records a set of sensor readings and keeps
		// track of the minimum and maximum values encountered.  The
		// IR_EMITTERS_ON argument means that the IR LEDs will be
		// turned on during the reading, which is usually what you
		// want.
		calibrate_line_sensors(IR_EMITTERS_ON);
		
		// Since our counter runs to 80, the total delay will be
		// 80*20 = 1600 ms.
		delay_ms(20);
	}*/
	set_motors(0,0);
}

void turn_left()
{
	set_motors(-42, 42);
	delay_ms(400);
	set_motors(0,0);
}

void go_forward()
{
}

void turn_right()
{
	set_motors(41, -41);
	delay_ms(400);
	set_motors(0,0);
}

void turn_back()
{
	set_motors(41, -41);
	delay_ms(800);
	set_motors(0,0);
}

// This is the main function, where the code starts.  All C programs
// must have a main() function defined somewhere.
int main()
{
	unsigned int sensors[5]; // an array to hold sensor values
	
	// set up the 3pi
	initialize();
	
	// This is the "main loop" - it will run forever.
	while(1)
	{
		// Get the position of the line.  Note that we must provide
		// the "sensors" argument to read_line() here, even though we
		// are not interested in the individual sensor readings.
		unsigned int position = read_line(sensors,IR_EMITTERS_ON);
		
		char strValues[30];
		int iter = 0;
		
		for ( int i = 0; i < 5; i++ )
		{
			char result[4];
			itoa( sensors[i], result );
			
			for ( int j = 0; j < 4; j++ )
			{
				strValues[iter++] = result[j];
			}
			
			strValues[iter++] = ' ';
		}
		
		delay_ms(500);
		print_from_program_space(strValues);
		
		/*if (sensors[0] > 300 || sensors[1] > 300 || sensors[2] > 300 || sensors[3] > 300)
		{
			set_motors(-25,25); // rotate left
		}
		else if (sensors[4] > 500)
		{
			set_motors(25,25); // go straight ahead
		}
		else
		{
			set_motors(25,-25); // rotate right
		}*/
		
		if(position < 1000)
		{
			// We are far to the right of the line: turn left.
			
			// Set the right motor to 100 and the left motor to zero,
			// to do a sharp turn to the left.  Note that the maximum
			// value of either motor speed is 255, so we are driving
			// it at just about 40% of the max.
			//set_motors(0,25);
			
			// Just for fun, indicate the direction we are turning on
			// the LEDs.
			//left_led(1);
			//right_led(0);
		}
		else if(position < 3000)
		{
			// We are somewhat close to being centered on the line:
			// drive straight.
			//set_motors(25,25);
			//left_led(1);
			//right_led(1);
		}
		else
		{
			// We are far to the left of the line: turn right.
			//set_motors(25,0);
			//left_led(0);
			//right_led(1);
		}
		// right most == sensors[4]
	}
}


//Ctrl+K+Dñ