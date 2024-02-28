#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHMotor.h>

// ----------- PORT AND MACRO DECLARATIONS -----------

// Macros
#define HALF_SPEED_VELOCITY 11 // Approximate velocity of the robot in inches per second (temporary until shaft encoding implemented)
#define LEFT 0 // Value used to represent a left turn in the turn() function
#define RIGHT 1 // Value used to represent a right turn in the turn() function
#define NINETY_DEGREE_TURN 0.5 // Time in seconds required to turn 90 degrees
#define REVERSE -1 // Value used to represent motion in reverse 

// Motor ports
FEHMotor right_motor(FEHMotor::Motor0, 9.0);
FEHMotor left_motor(FEHMotor::Motor2, 9.0);

// Analog inputs
AnalogInputPin cds_cell(FEHIO::P0_0);

// Digital inputs

// ----------- FUNCTIONS -----------

/*
    Reads the CdS cell sensor and returns the voltage reading
    PARAMS: N/A
    RETURN: 
        float cell_reading - voltage reading of CdS cell 
*/
float read_cds_sensor(){
    float cell_reading = cds_cell.Value();
    return(cell_reading);
}

/*
    Takes a given distance input in inches and returns the corresponding time for the robot to travel that distance in seconds. Calculation is based on the HALF_SPEED_VELOCITY macro above.
    PARAMS:
        int distance - distance to travel in inches
    RETURN:
        float time - time required to travel the input distance
*/
float distance_to_time(int distance){
    float time = distance / HALF_SPEED_VELOCITY;
    return(time);
}

// ----------- PROCEDURES -----------

/*
    Sets the motor percent for the left and right motors to 0%, stopping the robot. This function will be primarily used as a helper function for other functions.
    PARAMS: N/A
    RETURN: N/A
*/
void stop_motors(){
    left_motor.SetPercent(0);
    right_motor.SetPercent(0);
}

/*
    Sets the proper motor speed to enable turning for the specified duration of time.  
    PARAMS:
        float duration - total duration of turn in seconds
        int direction - direction of turn; left corresponds to direction = 0, right corresponds to direction = 1
    RETURN: N/A
*/
void turn(float duration, int direction){
    if(direction == 0){
        right_motor.SetPercent(50.);
        left_motor.SetPercent(-50.);
    }
    else {
        right_motor.SetPercent(-50.);
        left_motor.SetPercent(50.);
    }

    float finish_time = TimeNow() + duration;
    while(TimeNow() < finish_time){}
    stop_motors();
}

/*
    Sets the proper motor speed to move forward or in reverse depending on the value of direction.
    PARAMS:
        float duration - total duration of motion in seconds
        int direction - direction of motion, forward by default, but reverse if -1 is specified for direction
    RETURN: N/A
*/
void move(float duration, int direction=1){
    right_motor.SetPercent(50. * direction);
    left_motor.SetPercent(50. * direction);
    
    float total_time = TimeNow() + duration;
    while(TimeNow() < total_time){}
    stop_motors();
}

int main(void)
{
    LCD.Clear();

    Sleep(3.0);
    LCD.WriteLine("Starting...");
    // Wait until light signal is received
    while(read_cds_sensor() > 1.0){}
    move(3.0);
    Sleep(0.5);
    turn(NINETY_DEGREE_TURN, LEFT);
    Sleep(0.5);
    move(0.75);
    Sleep(0.5);
    turn(NINETY_DEGREE_TURN, RIGHT);
    Sleep(0.5);
    move(distance_to_time(25));

    move(distance_to_time(22), REVERSE);
    Sleep(0.5);
    turn(NINETY_DEGREE_TURN, LEFT);
    Sleep(0.5);
    move(0.90, REVERSE);
    Sleep(0.5);
    turn(NINETY_DEGREE_TURN, RIGHT);
    Sleep(0.5);
    move(3.0, REVERSE);

    // 11 in/sec
    LCD.WriteLine("balls");

	return 0;
}
