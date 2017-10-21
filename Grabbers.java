//Version 3.0 coded Oct. 21, 2017 by Marcus, Isaac, and Mathew.
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

@TeleOp(name = "Grabbers", group = "Grabbers")
//@Disabled
public class Grabbers extends OpMode {

    //Variable declarations

    Servo horizontal; //Servo that rotate's the grabber horizontally
    Servo openClose; //Sevo that opens and closes the two grabbers
    Servo rightGrabber; //Servo that controls the grabber on the right, with a reference point looking
    //at the openClose servo
    Servo leftGrabber; //Servo that controls the grabber on the left, with a reference point looking
    //at the openClose servo

    TouchSensor touchSensor1;
    TouchSensor touchSensor2;
    TouchSensor touchSensor3;

    boolean dpadUp = false; //boolean that is initially set to false, and becomes true if the dpad up is pressed
    boolean dpadDown = false; //boolean that is initially set to false, and becomes true if the dpad down is pressed
    double gamepadrightx; //double that represents the raw value of the x axis of the right stick of the gamepad being used
    boolean leftBumper = false; //boolean that is initially set to false, and becomes true if the left bumper is pressed
    double leftTrigger; //double that represents the raw value of the left trigger of the gamepad being used
    double openCloseValue = .5; //double that represents the position value to be applied to the openClose servo
    double conveyorValue = .5; //double that represents the postion value to be applied to the left and right grabbers
    double stillOpenCloseValue = .745;


    double adjustedRightX;
    double adjustedLeftTrigger;

    boolean touch1press = false;
    boolean touch2press = false;
    boolean touch3press = false;


    public void init() {
        //Servo configurations
        horizontal = hardwareMap.servo.get("s1");
        openClose = hardwareMap.servo.get("s2");
        rightGrabber = hardwareMap.servo.get("s3");
        leftGrabber = hardwareMap.servo.get("s4");

        touchSensor1 = hardwareMap.touchSensor.get("touch1");
        touchSensor2 = hardwareMap.touchSensor.get("touch2");
        touchSensor3 = hardwareMap.touchSensor.get("touch3");

        //Servo directions
        horizontal.setDirection(Servo.Direction.FORWARD);
        openClose.setDirection(Servo.Direction.FORWARD);
        rightGrabber.setDirection(Servo.Direction.REVERSE);
        leftGrabber.setDirection(Servo.Direction.FORWARD);

        //Initial powers
        horizontal.setPosition(.486);
        openClose.setPosition(.5);
        rightGrabber.setPosition(.5);
        leftGrabber.setPosition(.5);
    }

    public void loop() {

        //Turns touch sensors to booleans
        if (touchSensor1.isPressed())
        {
            touch1press = true;
        }
        else {
            touch1press = false;
        }

        if(touchSensor2.isPressed()) {
            touch2press = true;
        }
        else {
            touch2press = false;
        }

        if(touchSensor3.isPressed()) {
            touch3press = true;
        }
        else {
            touch3press = false;
        }


        //Set values of booleans
        if (gamepad1.dpad_up) {
            dpadUp = true;
        } else {
            dpadUp = false;
        }

        if (gamepad1.dpad_down) {
            dpadDown = true;
        } else {
            dpadDown = false;
        }

        if (gamepad1.left_bumper) {
            leftBumper = true;
        } else {
            leftBumper = false;
        }

        //Set gamepadright x and leftTriggger
        gamepadrightx = gamepad1.right_stick_x;
        leftTrigger = gamepad1.left_trigger;

        //As the Servo horizontal is continuous, its interval of values is (0,1), while
        //the interval of values for gamepadrightx is (-1,1).  Thus, we need to convert this value.
        adjustedRightX = .486 - gamepadrightx / 2;

        //The rawLeftTrigger value is in the interval (0,1).  However, for our purposes prssing the
        //left trigger means the grabbers are intaking, which represents values in the interval (0,.5), given
        //the right and left grabbers are continuous
        adjustedLeftTrigger = leftTrigger / 2 + .5;

        //Adjust openCloseValue as necesarry

        if (dpadUp && !dpadDown) {
            openCloseValue += .012;
        }
        if (dpadDown && !dpadUp) {
            openCloseValue -= .012;
        }

        openCloseValue = Range.clip(openCloseValue, .157, .745);

        if (leftTrigger > .05 && !leftBumper) {
            conveyorValue = adjustedLeftTrigger;
        } else if (leftTrigger < .05 && leftBumper) {
            conveyorValue = 0;
        }
        else if (leftTrigger< .05 && !leftBumper) {
            conveyorValue = .5;
        }
        else if (leftTrigger>.05 && leftBumper) {
            conveyorValue = .5;
        }
        else {
            conveyorValue = .5;
        }

        //Set powers
        horizontal.setPosition(adjustedRightX);
        if (touch1press && conveyorValue > .5)
        {
            conveyorValue = .5;
        }
        rightGrabber.setPosition(conveyorValue);
        leftGrabber.setPosition(conveyorValue);
        if (!touch1press)
        {
            openClose.setPosition(openCloseValue);
        }
        else
        {
            openClose.setPosition(stillOpenCloseValue);
        }
        //If fully extended one direction, not allowed to move further
        if(touch2press && adjustedRightX > .486)
        {
            horizontal.setPosition(.486);
        }
        else
        {
           horizontal.setPosition(adjustedRightX);
        }

        if(touch3press && adjustedRightX < .486)
        {
            horizontal.setPosition(.486);
        }
        else
        {
            horizontal.setPosition(adjustedRightX);
        }



        telemetry.addData("open close value", openCloseValue);
        telemetry.addData("adj right x", adjustedRightX);
        telemetry.update();

    }
}
