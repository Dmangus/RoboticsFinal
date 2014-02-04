#include <pololu/3pi.h>

/***********
 * Globals *
 ***********/
 
const int max = 60; // the maximum speed
unsigned int sensors[5];


//Serial communication Globals
char buffer[100];
char command[256];
unsigned char receive_buffer_position  = 0;
volatile int error = 0;
char ack[1] = {0x06};
char nak[1] = {0x15};

void follow_segment()
{
	int last_proportional = 0;
	long integral=0;

	while(1)
	{
		// Normally, we will be following a line.  The code below is
		// similar to the 3pi-linefollower-pid example, but the maximum
		// speed is turned down to 60 for reliability.

		// Get the position of the line.
		unsigned int sensors[5];
		unsigned int position = read_line(sensors,IR_EMITTERS_ON);

		// The "proportional" term should be 0 when we are on the line.
		int proportional = ((int)position) - 2000;

		// Compute the derivative (change) and integral (sum) of the
		// position.
		int derivative = proportional - last_proportional;
		integral += proportional;

		// Remember the last position.
		last_proportional = proportional;

		// Compute the difference between the two motor power settings,
		// m1 - m2.  If this is a positive number the robot will turn
		// to the left.  If it is a negative number, the robot will
		// turn to the right, and the magnitude of the number determines
		// the sharpness of the turn.
		int power_difference = proportional/20 + integral/10000 + derivative*3/2;

		// Compute the actual motor settings.  We never set either motor
		// to a negative value.
		
		if(power_difference > max)
			power_difference = max;
		if(power_difference < -max)
			power_difference = -max;
		
		if(power_difference < 0)
			set_motors(max+power_difference,max);
		else
			set_motors(max,max-power_difference);

		// We use the inner three sensors (1, 2, and 3) for
		// determining whether there is a line straight ahead, and the
		// sensors 0 and 4 for detecting lines going to the left and
		// right.

		if(sensors[1] < 100 && sensors[2] < 100 && sensors[3] < 100)
		{
			// There is no line visible ahead, and we didn't see any
			// intersection.  Must be a dead end.
			set_motors(50,50);
			delay_ms(220);
			set_motors(0,0);
			return;
		}
		else if(sensors[0] > 200 || sensors[4] > 200)
		{
			// Found an intersection.
			set_motors(50,50);
			delay_ms(220);
			set_motors(0,0);
			return;
        }
	}
}

char read_next_byte()
{
    //Set Timeout value
	long timeout = get_ms() + 1000;
    
    //Wait till either we get a serial byte or we time out
	while((serial_get_received_bytes() == receive_buffer_position) && (timeout >= get_ms()) );		
    
    //If we Timeout return an ERROR, else return the received byte and increase our buffer position
	if(get_ms() >= timeout)
	{
		error = 1;
		return 0xFF;
	}
	else
	{
		char ret = buffer[receive_buffer_position++];
        buffer[receive_buffer_position-1] = 0x00;
        //This is what makes a ring buffer a ring buffer, when it hits the end of the buffer it starts over
	    if (receive_buffer_position == sizeof(buffer))
			receive_buffer_position = 0;
		
		return ret;	
	}
}

void auto_calibrate()
{
	time_reset();
	set_motors(60, -60);
	while(get_ms() < 250)
        calibrate_line_sensors(IR_EMITTERS_ON);
    
	set_motors(-60, 60);
	while(get_ms() < 750)
        calibrate_line_sensors(IR_EMITTERS_ON);
    
	set_motors(60, -60);
	while(get_ms() < 1000)
        calibrate_line_sensors(IR_EMITTERS_ON);
    
	set_motors(0, 0);
}

void initialize()
{
	// Clear The Screen and print the battery voltage
    clear();
    print("Battery");
    lcd_goto_xy(0,1);
    print_long(read_battery_millivolts());
    print("mV");

    // Initialize Serial Baud rate
	serial_set_baud_rate(9600);

	// Start receiving bytes in the ring buffer.
	serial_receive_ring(buffer, sizeof(buffer));
	serial_set_mode(SERIAL_AUTOMATIC);
    
	// This must be called at the beginning of 3pi code, to set up the
	// sensors.  We use a value of 2000 for the timeout, which
	// corresponds to 2000*0.4 us = 0.8 ms on our 20 MHz processor.
	pololu_3pi_init(2000);
}

void turn(char dir)
{
	switch(dir)
	{
		case 'L':  // Turn left
            set_motors(-80,80);
            delay_ms(100);
			while(1)
			{
				read_line(sensors,IR_EMITTERS_ON);
				if(sensors[1] > 100 && sensors[2] > 200 && sensors[3] > 100)
					break;
			}
		break;
		case 'R':  // Turn right
            set_motors(80,-80);
            delay_ms(100);
			while(1)
			{
				read_line(sensors,IR_EMITTERS_ON); 
				if(sensors[1] > 100 && sensors[2] > 200 && sensors[3] > 100)
					break;
			}
		break;
		case 'B':  // Turn around
            set_motors(80,-80);
            delay_ms(300);
			while(1)
			{
				read_line(sensors,IR_EMITTERS_ON); 
				if(sensors[1] > 100 && sensors[2] > 200 && sensors[3] > 100)
					break;
			}
		break;
		case 'S':
            // Don't do anything!
		break;
	}
	set_motors(0,0);
}

int main()
{	
	// Does initialization of the 3pi
	initialize();
	int i,j;
	uint8_t toggle = 1;

	// Loop Forever Loop, Loops Forever
	while(1)
	{
		// Error Detection and escape loop
		while(1) 
		{
    		// Wait for STX, and print out battery voltage so we can keep track of it
            // @TODO if battery voltage < 4000 send out a LOW BATTERY message over serial and turn the 3pi OFF
			j = 0; //j == Number of bytes in the command that are usable
			while((command[j] = read_next_byte()) != 0x02)
            {
                clear();
                print("Battery");
                lcd_goto_xy(0,1);
                print_long(read_battery_millivolts());
                print("mV");
				toggle = !toggle;
				green_led(toggle);
            }
			
			// This clears any errors that were likely set waiting for the STX of a new command to come in as well as any error that occured while getting a command eariler
			error = 0;
			j++;

			// Datalength
			command[j++] = read_next_byte();
			if(error == 1)
				break;

			// Command
			command[j++] = read_next_byte();
			if(error == 1)
				break;
            
            // If there is actual Data...
			if(command[1] > 0)
			{
				i = (int)command[1];
				while(i--)
				{
					command[j++] = read_next_byte();
					if(error == 1)
						break;
				}
			}

			// ETX
			command[j++] = read_next_byte();
			if(error == 1)
				break;
				
			// Checksum
			command[j] = read_next_byte();
			if(error == 1)
				break;

			// calculate robot side checksum
			char LRC = 0;
			for(int p=0; p<j; p++)
				LRC ^= command[p];
            
            // If the Calculated checksum isn't the same as the checksum sent by the control center, there is an error in the data
			if(LRC != command[j])
				break;
			

			/****************************************************
             * Complete/(Most Likely Correct) Command Received! *
             ****************************************************/
            
			// Now we can finaly figure out what to do with this command since it has been tested, verified
			switch(command[2])
			{ 
				case 0x00: // CALIBRATE LINE SENSORS
					auto_calibrate();
			    break;				    
				case 0x01: // FOLLOW SEGMENT
					follow_segment();
				break;		    
				case 0x02: // TURN
					turn(command[3]);
				break;
				case 0x03: // STOP
					set_motors(0,0);
				break; 
                default:  
                    // @TODO: If an unknown command was received send Error Message
					// serial_send_blocking((char *)nak, 1);
				break;
			}

			//Aknowledge that we got the command and are done acting on it
			delay_ms(50);
			serial_send_blocking((char *)ack, 1);
		}
        
        /*******************************************************
         * GOT AN ERROR, SEND A NAK AND START THE COMMAND OVER *
         *******************************************************/
		delay_ms(50);
    	serial_send_blocking((char *)nak, 1);
	}
}
