#include <math.h>
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHMotor.h>
#include <FEHRCS.h>
#include <FEHServo.h>

// ----------- PORT AND MACRO DECLARATIONS -----------

// Macros
#define LEFT 0 // Value used to represent a left turn in the turn() function
#define RIGHT 1 // Value used to represent a right turn in the turn() function
#define REVERSE -1 // Value used to represent motion in reverse
#define FORWARD 1 // Value used to represent motion forward
#define COUNTS_PER_REVOLUTION 318 // Number of counts that correspond to a full motor revolution for IGWAN motors 
#define RADIUS_OF_TURN 4 // Radius of robot turn in inches measured from the middle of the wheel to the center of the chassis
#define TEAM_ID "B5rhNym2B" // Team identifier used for RCS system 
#define SERVO_MIN 1291 // Minimum compensation value for the servo motor, run TouchCalibrate() to obtain
#define SERVO_MAX 2313 // Maximum compensation value for the servo motor, run TouchCalibrate() to obtain
#define COLOR_THRESHOLD 1.7 // Threshold used for differentiating between red and blue lights, > 1.7 corresponds to blue and < 1.7 corresponds to red

// Motor ports
FEHMotor right_motor(FEHMotor::Motor0, 9.0);
FEHMotor left_motor(FEHMotor::Motor2, 9.0);

// Servo ports
FEHServo servo_arm(FEHServo::Servo0);

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
void reset_motor_counts(){
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

    reset_motor_counts();

    float turn_radius_radians = deg_to_rads(angle);
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

    reset_motor_counts();

    while(calculate_distance(right_encoder.Counts()) <= distance && calculate_distance(left_encoder.Counts()) <= distance){}
    stop_motors();
    Sleep(0.5);
}

void move_failsafe(float distance, float failsafe_duration, int direction=1, float speed=40.){
    right_motor.SetPercent(speed * direction);
    left_motor.SetPercent(speed * direction);

    reset_motor_counts();

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

    reset_motor_counts();

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
    if(voltage > COLOR_THRESHOLD){
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
    Continuously outputs the voltage readings of the CdS cell for calibration purposes
    PARAMS: N/A
    RETURN: N/A
*/
void calibrate_cds(){
    // adjust the COLOR_THRESHOLD constant for calibration
    while(true){
        float reading = read_cds_sensor();
        LCD.WriteLine(reading);
        if(reading > COLOR_THRESHOLD){
            LCD.WriteLine("BLUE BASED ON CURRENT THRESHOLD");
        }
        else{
            LCD.WriteLine("RED BASED ON CURRENT THRESHOLD");
        }
        Sleep(0.5);
        LCD.Clear();
    }
}

/*
    Adjusts the position of the servo motor to the input angle.
    PARAMS:
        angle - float value representing the desired angle of the servo motor
    RETURN: N/A
*/
void move_servo(float angle){
    // LEFT MOST PORT WITH BLACK WIRE ON TOP
    servo_arm.SetDegree(angle);
}


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

    // ---------- UNCOMMENT THIS TO CALIBRATE ----------
    // calibrate_cds();

    init();

    while(read_cds_sensor() > 2.0){}
    
    /* ---------- LUGGAGE DROP ---------- */
    move_failsafe(3., 0.75, REVERSE);
    move(1., FORWARD);
    turn(40., RIGHT);
    move(18., FORWARD);
    turn(2.5, LEFT);
    move(19.,FORWARD);
    turn(87., LEFT);
    move(12.25, FORWARD);
    turn(85., RIGHT);
    move(1., REVERSE);
    for(float i = 180.; i >= 110.; i -= 10){
        move_servo(i);
        Sleep(0.1);    
    }
    move_servo(180.);
    Sleep(0.2);
    move(4.5, FORWARD);
    
    /* ---------- LIGHT READING ---------- */
    
    turn(87., LEFT);
    move_failsafe(999., 2., FORWARD);
    move(8.75, REVERSE);
    turn(83., RIGHT);
    move_to_light(FORWARD);
    int light_color = read_light_color();
    move(2.0, REVERSE);
    turn(80.0, RIGHT);
    
    /* ---------- BOARDING PASS BUTTONS ---------- */
    
    // RED
    if(light_color == 1){
        move(6.25, FORWARD);
        turn(83.0, LEFT);
        move(6.5, FORWARD, 55.);
        move(7., REVERSE);
        turn(83., LEFT);
        move(7.5, FORWARD);
    }
    // BLUE
    else {
        move(9.0, FORWARD);
        turn(85.0, LEFT);
        move(5.5, FORWARD, 55.);
        move(7., REVERSE);
        turn(81., LEFT);
        move(11.5, FORWARD);
    }

    /* ---------- PASSPORT STAMP ---------- */
    move_servo(0.);
    move(6.25, REVERSE);
    Sleep(1.5);
    move_servo(135.);
    turn(40., LEFT);
    turn(15., RIGHT);

    /* ---------- FUEL LEVERS ---------- */
    move_failsafe(999., 2.75, FORWARD);
    move_servo(180.);
    move(5.0, REVERSE);
    turn(81.5, RIGHT);
    int correctLever = RCS.GetCorrectLever();
    // int correctLever = 2;
    move(28.5, REVERSE);
    if(correctLever == 0){
        //  LEFT - A
        turn(83., LEFT);
        move(6.5, REVERSE);
        turn(83., RIGHT);
    }
    else if(correctLever == 1){
        //  MIDDLE - A1
        turn(83., LEFT);
        move(3, REVERSE);
        turn(83., RIGHT);
        move(1., FORWARD);
    }
    else{
        //  RIGHT - B
    }
    move_servo(45.);
    Sleep(.2);
    move(3.5, FORWARD);
    move_servo(0.);
    Sleep(5.);
    move(2.75, REVERSE);
    move_servo(60.);
    Sleep(.3);

    /* ---------- FINAL BUTTON ---------- */
    move(2., FORWARD);
    move_servo(180.);
    move(4., REVERSE);
    turn(83., RIGHT);
    move_failsafe(999., 2.5, FORWARD);
    move(3.5, REVERSE);
    turn(83., RIGHT);
    move(16., FORWARD, 45.);
    turn(45., LEFT);
    move(4., FORWARD, 60.);

    return 0;
}