//Version 2.0 coded Oct. 7, 2017 by Marcus and Isaac.
//Designed to test the functionality of block grabber prototype
//So far so good!

//package declaration
package org.firstinspires.ftc.teamcode;

//import statements
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.qualcomm.robotcore.util.Range.clip;

/**
 * Created by MarcusLeher on 06/10/2017.
 */

@TeleOp(name = "Lifters", group = "Lifters")
//@Disabled
public class Lifters extends OpMode {
    // Declarations

    // DCMotors
    DcMotor rightLift;     //right lift motor front
    DcMotor leftLift;      //left lift motor front

    //double timeOne = 0;
    double rightEncoderCounts;
    double leftEncoderCounts;

    double rightLiftPower;
    double leftLiftPower;

    //final double ENCODER_CPR = 134.4; //NeveRest Orbital 20
    //final double GEAR_RATIO = 1;

    double baseMotorPower;

    double percentDifference;


    public void init() { // use hardwaremap here instead of hwmap or ahwmap provided in sample code

        // Motor configurations in the hardware map
        rightLift = hardwareMap.dcMotor.get("liftMotorRight");
        leftLift = hardwareMap.dcMotor.get("liftMotorLeft");

        // Motor directions: set forward/reverse
        rightLift.setDirection(REVERSE);
        leftLift.setDirection(FORWARD);

        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void loop() {
        baseMotorPower = gamepad1.right_stick_y;

        rightEncoderCounts = rightLift.getCurrentPosition();
        leftEncoderCounts = leftLift.getCurrentPosition();

        percentDifference = (rightEncoderCounts - leftEncoderCounts)/(rightEncoderCounts + leftEncoderCounts);

        if (baseMotorPower > .05)
        {
            rightLiftPower = baseMotorPower - percentDifference/2;
            leftLiftPower = baseMotorPower;
            rightLiftPower = Range.clip(rightLiftPower, .05, .95);
            leftLiftPower = Range.clip(leftLiftPower, .05, .95);
        }
        else if (baseMotorPower < .05)
        {
            rightLiftPower = baseMotorPower + percentDifference/2;
            leftLiftPower = baseMotorPower;
            rightLiftPower = Range.clip(rightLiftPower, -.95, -.05);
            leftLiftPower = Range.clip(leftLiftPower, -.95, -.05);
        }
        else
        {
            rightLiftPower = 0;
            leftLiftPower = 0;
        }

        rightLift.setPower(rightLiftPower);
        leftLift.setPower(leftLiftPower);
    }
}


