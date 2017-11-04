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

@TeleOp(name = "GrabbersCombo", group = "GrabbersCombo")
//@Disabled
public class GrabbersCombo extends OpMode {

    Servo horizontalTop; //Servo that rotate's the grabber horizontally
    Servo openCloseTop; //Sevo that opens and closes the two grabbers
    Servo rightGrabberTop; //Servo that controls the grabber on the right, with a reference point looking
    //at the openClose servo
    Servo leftGrabberTop; //Servo that controls the grabber on the left, with a reference point looking
    //at the openClose servo

    Servo horizontalBottom; //Servo that rotate's the grabber horizontally
    Servo openCloseBottom; //Sevo that opens and closes the two grabbers
    Servo rightGrabberBottom; //Servo that controls the grabber on the right, with a reference point looking
    //at the openClose servo
    Servo leftGrabberBottom; //Servo that controls the grabber on the left, with a reference point looking
    //at the openClose servo

    TouchSensor touchSensorTop;
    TouchSensor touchSensorBottom;

    //Top
    boolean dpadUp = false; //boolean that is initially set to false, and becomes true if the dpad top is pressed
    boolean dpadDown = false; //boolean that is initially set to false, and becomes true if the dpad bottom is pressed
    double gamepadrightx; //double that represents the raw value of the x axis of the left stick of the gamepad being used
    boolean leftBumper = false; //boolean that is initially set to false, and becomes true if the left bumper is pressed
    double leftTrigger; //double that represents the raw value of the left trigger of the gamepad being used
    double openCloseValueTop = .5; //double that represents the position value to be applied to the openClose top servo
    double conveyorValueTop = .5; //double that represents the postion value to be applied to the left and right grabbers on top
    double stillOpenCloseValueTop = .745;  //double that represents the value of the top open close servo when it is still

    //Variable declarations


    TouchSensor touchSensorTop;
    TouchSensor touchSensorBottom;

    //Bottom
    boolean dpadRight = false; //boolean that is initially set to false, and becomes true if the dpad right is pressed
    boolean dpadLeft = false; //boolean that is initially set to false, and becomes true if the dpad left is pressed
    double gamepadrighty; //double that represents the raw value of the y axis of the right stick of the gamepad being used
    boolean rightBumper = false; //boolean that is initially set to false, and becomes true if the right bumper is pressed
    double rightTrigger; //double that represents the raw value of the right trigger of the gamepad being used
    double openCloseValueBottom = .5; //double that represents the position value to be applied to the openClose bottom servo
    double conveyorValueBottom = .5; //double that represents the postion value to be applied to the left and right grabbers on bottom
    double stillOpenCloseValue = .745;   //double that represents the value of the bottom open close servo when it is still

    boolean touchTopPress = false;
    boolean touchBottomPress = false;

    double adjustedRightX;
    double adjustedLeftTrigger;

    public void init () {
        //Servo configurations
        horizontalTop = hardwareMap.servo.get("s1");
        openCloseTop = hardwareMap.servo.get("s2");
        rightGrabberTop = hardwareMap.servo.get("s4");
        leftGrabberTop = hardwareMap.servo.get("s3");

        //Servo configurations
        horizontalBottom = hardwareMap.servo.get("s5");
        openCloseBottom = hardwareMap.servo.get("s6");
        rightGrabberBottom = hardwareMap.servo.get("s8");
        leftGrabberBottom = hardwareMap.servo.get("s7");

        touchSensorTop = hardwareMap.touchSensor.get("touch1");
        touchSensorBottom = hardwareMap.touchSensor.get("touch2");


        //Servo directions
        horizontalTop.setDirection(Servo.Direction.FORWARD);
        openCloseTop.setDirection(Servo.Direction.FORWARD);
        rightGrabberTop.setDirection(Servo.Direction.REVERSE);
        leftGrabberTop.setDirection(Servo.Direction.FORWARD);

        horizontalBottom.setDirection(Servo.Direction.FORWARD);
        openCloseBottom.setDirection(Servo.Direction.FORWARD);
        rightGrabberBottom.setDirection(Servo.Direction.REVERSE);
        leftGrabberBottom.setDirection(Servo.Direction.FORWARD);

        //Initial powers
        horizontalTop.setPosition(.486);
        openCloseTop.setPosition(.5);
        rightGrabberTop.setPosition(.5);
        leftGrabberTop.setPosition(.5);

        horizontalBottom.setPosition(.486);
        openCloseBottom.setPosition(.5);
        rightGrabberBottom.setPosition(.5);
        leftGrabberBottom.setPosition(.5);
        }

    public void loop() {
        if (touchSensorTop.isPressed())
        {
            touchTopPress = true;
        }
        else
        {
            touchTopPress = false;
        }

        if (touchSensorBottom.isPressed())
        {
            touchBottomPress = true;
        }
        else
        {
            touchBottomPress = false;
        }



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

        if (gamepad1.dpad_right) {
            dpadRight = true;
        } else {
            dpadRight = false;
        }

        if (gamepad1.dpad_down) {
            dpadLeft = true;
        } else {
            dpadLeft = false;
        }

        if (gamepad1.right_bumper) {
            rightBumper = true;
        } else {
            rightBumper = false;
        }






    }


}
