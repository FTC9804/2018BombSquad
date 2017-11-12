package org.firstinspires.ftc.teamcode;

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

/**
 * Created by WilderBuchanan on 11/11/17.
 */

public class RobotMain extends OpMode {
    // Variables
    double rightPadY;
    double leftPadX;
    double rightPadX;

    Drive drive;

    // DCMotors
    DcMotor rightMotor, leftMotor, frontMotor, backMotor;

    /* Initialize standard Hardware interfaces */
    public void init() { // use hardwaremap here instead of hwmap or ahwmap provided in sample code
        // Motor configurations in the hardware map
        DcMotor newRightMotor = hardwareMap.dcMotor.get("m1");
        DcMotor newLeftMotor = hardwareMap.dcMotor.get("m2");
        DcMotor newFrontMotor = hardwareMap.dcMotor.get("m4");
        DcMotor newBackMotor = hardwareMap.dcMotor.get("m3");
        drive = new Drive (newRightMotor, newLeftMotor, newFrontMotor, newBackMotor);

    }

    public void loop () {

        rightPadY = gamepad2.right_stick_y;
        rightPadX = gamepad2.right_stick_x;
        leftPadX = -(gamepad2.left_stick_x);

        // If pads are moved
        boolean leftPadXOn = Math.abs (leftPadX) > .05;
        boolean rightPadXOn = Math.abs (rightPadX) > .05;
        boolean rightPadYOn = Math.abs (rightPadY) > .05;

        // Combine rotation and movement
        if (leftPadXOn && !rightPadXOn && !rightPadYOn)
        {
            drive.turn(leftPadX);
        }
        else
        {
            drive.move (rightPadX, rightPadY);
        }

        drive.run();
    } // end loop
} // end class
