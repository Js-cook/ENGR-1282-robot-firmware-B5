#include <math.h>
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHMotor.h>
#include <FEHRCS.h>
#include <FEHServo.h>

// ----------- PORT AND MACRO DECLARATIONS -----------

// Macros
#define HALF_SPEED_VELOCITY 11 // Approximate velocity of the robot in inches per second (temporary until shaft encoding implemented)
#define LEFT 0 // Value used to represent a left turn in the turn() function
#define RIGHT 1 // Value used to represent a right turn in the turn() function
#define NINETY_DEGREE_TURN 0.5 // Time in seconds required to turn 90 degrees
#define REVERSE -1 // Value used to represent motion in reverse
#define FORWARD 1 // Value used to represent motion forward
#define COUNTS_PER_REVOLUTION 318 // Number of counts that correspond to a full motor revolution for IGWAN motors 
#define RADIUS_OF_TURN 4 // Radius of robot turn in inches measured from the middle of the wheel to the center of the chassis
#define TEAM_ID "B5rhNym2B" // Team identifier used for RCS system 
// UPDATE THESE WHEN READY
#define SERVO_MIN 0 // Minimum compensation value for the servo motor, run TouchCalibrate() to obtain
#define SERVO_MAX 999 // Maximum compensation value for the servo motor, run TouchCalibrate() to obtain

// Motor ports
FEHMotor right_motor(FEHMotor::Motor0, 9.0);
FEHMotor left_motor(FEHMotor::Motor2, 9.0);

// Servo ports
FEHServo servo_arm(FEHServo::Servo2); // replace with actual servo motor port

// Analog inputs
AnalogInputPin cds_cell(FEHIO::P1_0);

// Digital inputs

// Digital encodings
DigitalEncoder right_encoder(FEHIO::P0_1);
DigitalEncoder left_encoder(FEHIO::P0_2);

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
    Calculates the distance traveled based on the number of counts input.
    PARAMS:
        int counts - motor counts
    RETURN: 
        float distance - distance traveled in inches 
*/
float calculate_distance(int counts){
    // s = (2 * pi * wheelRadius * counts) / countsPerRevolution
    float distance = (2 * M_PI * 1.5 * counts) / COUNTS_PER_REVOLUTION;
    return(distance);
} 

/*
    Converts degrees to radians
    PARAMS:
        float degrees - degrees
    RETURN: 
        float rads - radians
*/
float deg_to_rads(float degrees){
    float rads = degrees * (M_PI / 180.0);
    return(rads);
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
    Resets the counts for both the left and right motors. Another helper function similar to stop_motors()
    PARAMS: N/A
    RETURN: N/A
*/
void resetMotorCounts(){
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
}

/*
    Sets the proper motor speed to enable turning for the specified angle.  
    PARAMS:
        float angle - angle of turn in degrees 
        int direction - direction of turn; left corresponds to direction = 0, right corresponds to direction = 1
    RETURN: N/A
*/
void turn(float angle, int direction){
    if(direction == 0){
        right_motor.SetPercent(40.);
        left_motor.SetPercent(-40.);
    }
    else {
        right_motor.SetPercent(-40.);
        left_motor.SetPercent(40.);
    }

    resetMotorCounts();

    float turn_radius_radians = deg_to_rads(angle);
    // derived from arc length formula double check this
    while(calculate_distance(right_encoder.Counts()) <= (RADIUS_OF_TURN * turn_radius_radians)){}
    stop_motors();
    Sleep(0.5);
}


/*
    Sets the proper motor speed to move forward or in reverse depending on the value of direction.
    PARAMS:
        float distance - total distance of motion in inches
        int direction - direction of motion, forward by default, but reverse if -1 is specified for direction
        float speed - motor speed as a percentage
    RETURN: N/A
*/
void move(float distance, int direction=1, float speed=40.){
    right_motor.SetPercent(speed * direction);
    left_motor.SetPercent(speed * direction);

    resetMotorCounts();

    // Ideally motor counts should be equal but this might have to be adjusted in the future
    // int prevCounts = -1;
    while(calculate_distance(right_encoder.Counts()) <= distance && calculate_distance(left_encoder.Counts()) <= distance){}
    stop_motors();
    Sleep(0.5);
}

void move_failsafe(float distance, float failsafe_duration, int direction=1, float speed=40.){
    right_motor.SetPercent(speed * direction);
    left_motor.SetPercent(speed * direction);

    resetMotorCounts();

    float time = TimeNow();
    while((calculate_distance(right_encoder.Counts()) <= distance && calculate_distance(left_encoder.Counts()) <= distance) && TimeNow() < time + failsafe_duration){}
    stop_motors();
    Sleep(0.5);
}

/*
    Modified move() to stop when ticket booth light is reached.
    PARAMS:
        int direction - direction of motion, forward by default, but reverse if -1 is specified for direction
    RETURN: N/A
*/
void move_to_light(int direction=1){
    right_motor.SetPercent(40. * direction);
    left_motor.SetPercent(40. * direction);

    resetMotorCounts();

    while(read_cds_sensor() > 2.2){}
    stop_motors();
    Sleep(0.5);
}

/*
    Reads the color of the ticket booth light and displays the color to the LCD screen.
    PARAMS: N/A
    RETURN:
        light_color - int value representing the color of the light, 1 corresponds to red, 2 corresponds to blue
*/
int read_light_color(){
    int light_color;
    float voltage;
    float time = TimeNow();
    while(TimeNow() < time + 1){
        voltage = read_cds_sensor();
    }

    LCD.Clear();

    time = TimeNow();
    if(voltage > 1.35){
        // color is blue
        LCD.WriteLine("Blue");
        LCD.SetFontColor(BLUE);
        LCD.FillRectangle(0, 0, 319, 239);
        while(TimeNow() < time + 0.5){}
        light_color = 2;
    }
    else {
        // color is red
        LCD.WriteLine("Red");
        LCD.SetFontColor(RED);
        LCD.FillRectangle(0, 0, 319, 239);
        while(TimeNow() < time + 0.5){}
        light_color = 1;
    }
    return(light_color);
}

/*
    Navigates to a fuel lever designated by its numerical id. Value of switch_id fetched from GetCorrectLever() from RCS module.
    PARAMS:
        switch_id - int value representing the correct lever
    RETURN: N/A
*/
void navigate_to_switch(int switch_id){
    switch(switch_id){
        case 0:
            LCD.WriteLine("Left");
            move(20.25);
            turn(77., LEFT);
            move(6.25, 1, 65.);
            break;
        case 1:
            LCD.WriteLine("Middle");
            move(24.0);
            turn(77., LEFT);
            move(6.25, 1, 65.);
            break;
        case 2:
            LCD.WriteLine("Right");
            move(28.);
            turn(77., LEFT);
            move(6.25, 1, 65.);
            break;
        default:
            LCD.WriteLine("I no no wanna :(");
            break;
    }
}

/*
    Adjusts the position of the servo motor to the input angle.
    PARAMS:
        angle - float value representing the desired angle of the servo motor
    RETURN: N/A
*/
void move_servo(float angle){
    servo_arm.SetDegree(angle);
}

// void reset_servo()

/*
    Initializes parameters and settings for the robot. Called once at program start
    PARAMS: N/A
    RETURN: N/A
*/
void init(){
    LCD.Clear();
    RCS.InitializeTouchMenu(TEAM_ID);

    servo_arm.SetMin(SERVO_MIN);
    servo_arm.SetMax(SERVO_MAX);
}

int main(void)
{
    init();

    while(read_cds_sensor() > 2.0){}

    move(36., FORWARD);
    turn(87., LEFT);
    move(4.75, FORWARD);
    turn(80., LEFT);
    move_failsafe(9999., 0.5, REVERSE);
    turn(3., LEFT);
    move_failsafe(9999., 1.5, REVERSE);
    move_failsafe(9999., 2.5, FORWARD);
    // left_motor.SetPercent(-40.);
    // right_motor.SetPercent(-40.);
    // move(11., REVERSE, 75.);
    // move(11., FORWARD, 75.);

	return 0;
}
