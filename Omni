//Version 2.0 coded Sep. 30, 2017 by Steve, Isaac, and Marcus.
//Designed to test the functionality of OmniDrive

//package declaration
package org.firstinspires.ftc.teamcode;

//import statements
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.qualcomm.robotcore.util.Range.clip;


@TeleOp(name = "OmniDrive", group = "OmniDriveBot")
//@Disabled
public class Omni extends OpMode {

    // Declarations

    // DCMotors
    DcMotor rightMotor;     //right drive motor front
    DcMotor leftMotor;      //left drive motor front
    DcMotor topMotor;       //right drive motor back
    DcMotor bottomMotor;    //left drive motor back

    // Variables
    double gamepadrighty;
    double gamepadleftx;
    double gamepadrightx;
    double leftpower;
    double rightpower;
    double toppower;
    double bottompower;

    /* Initialize standard Hardware interfaces */
    public void init() { // use hardwaremap here instead of hwmap or ahwmap provided in sample code

        // Motor configurations in the hardware map
        rightMotor = hardwareMap.dcMotor.get("m1");
        leftMotor = hardwareMap.dcMotor.get("m2");
        topMotor = hardwareMap.dcMotor.get("m3");
        bottomMotor = hardwareMap.dcMotor.get("m4");

        // Motor directions: set forward/reverse
        rightMotor.setDirection(REVERSE);
        leftMotor.setDirection(FORWARD);
        topMotor.setDirection(REVERSE);
        bottomMotor.setDirection(FORWARD);
    }

    public void loop() {

        // Values from joystick given to variables
        gamepadrighty = gamepad1.right_stick_y;
        gamepadrightx = gamepad1.right_stick_x;
        gamepadleftx = -(gamepad1.left_stick_x);


        // Rotation as priority over standard robot movement
        if(gamepadleftx > .05 || gamepadleftx < -.05)
        {

            leftpower = gamepadleftx;
            rightpower = -(gamepadleftx);
            toppower = -gamepadleftx;
            bottompower = gamepadleftx;

        }   // end if statement

        // standard movement of the robot
        else
        {
            //forward backwards
            leftpower = gamepadrighty;
            rightpower = gamepadrighty;

            //strafe
            toppower = gamepadrightx;
            bottompower = gamepadrightx;

        }   // end else statement


        // clip power to ensure we are not giving too high a value
        toppower = clip(toppower, -1, 1);
        bottompower = clip(bottompower, -1, 1);
        leftpower = clip(leftpower, -1, 1);
        rightpower = clip(rightpower, -1, 1);


        //set motor powers
        leftMotor.setPower(leftpower);
        rightMotor.setPower(rightpower);
        topMotor.setPower(toppower);
        bottomMotor.setPower(bottompower);

    }   // end loop
}   //end class
