// The 3pi include file must be at the beginning of any program that
// uses the Pololu AVR library and 3pi.
#include <pololu/3pi.h>

#include <avr/pgmspace.h>

void initialize()
{
	delay_ms(2000);
	
	pololu_3pi_init(2000);
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
	int moving = 1;
	int iter = 0;
	
	// set up the 3pi
	initialize();
	
	test_sensors(sensors);
	//mainRobotLogic(sensors);
}

void test_sensors(unsigned int sensors[5]){
	int moving = 1;
	int iter = 0;
	while(1)
	{
		read_line_sensors( sensors, IR_EMITTERS_ON );
		for(int i =0;i<5;i++)
		{
			print_long(sensors[i]);
			delay_ms(500);
			clear();
		}
		play(">>a32");
	}

}


void mainRobotLogic(unsigned int sensors[5])
{
	int moving = 1;
	int iter = 0;
	while(1)
	{
		read_line_sensors( sensors, IR_EMITTERS_ON );
		//test_sensors(sensors);
		if ( sensors[1] > 800 || sensors[2] > 800 || sensors[3] > 800 )
		{
			if ( moving )
			{
				iter++;
				play(">>a32");
				moving = 0;
				
				delay_ms(300);
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
		
		if(iter<7){
			set_motors( 20, 20 );
		}
		
	}
}
//Ctrl+K+D