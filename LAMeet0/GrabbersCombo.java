// Version 2.0 coded Oct. 7, 2017 by Marcus,Steve, and Isaac.
// Designed to test the functionality of block grabber prototype
// So far so good!

// package declaration
package org.firstinspires.ftc.teamcode;

// import statements
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
// @Disabled
public class GrabbersCombo extends OpMode {

    //Servo horizontalTop; // Servo that rotate's the grabber horizontally
    Servo openCloseTop; // Sevo that opens and closes the two grabbers
    Servo rightGrabberTop; // Servo that controls the grabber on the right, with a reference point looking
    // at the openClose servo
    Servo leftGrabberTop; // Servo that controls the grabber on the left, with a reference point looking
    // at the openClose servo

    //Servo horizontalBottom; // Servo that rotate's the grabber horizontally
    Servo openCloseBottom; // Sevo that opens and closes the two grabbers
    Servo rightGrabberBottom; // Servo that controls the grabber on the right, with a reference point looking
    // at the openClose servo
    Servo leftGrabberBottom; // Servo that controls the grabber on the left, with a reference point looking
    // at the openClose servo

    // Top
    boolean dpadUpTop = false; // boolean that is initially set to false, and becomes true if the dpad top is pressed
    boolean dpadDownTop = false; // boolean that is initially set to false, and becomes true if the dpad bottom is pressed
    double gamepadrightxTop; // double that represents the raw value of the x axis of the left stick of the gamepad being used
    boolean leftBumperTop = false; // boolean that is initially set to false, and becomes true if the left bumper is pressed
    double leftTriggerTop; // double that represents the raw value of the left trigger of the gamepad being used
    double openCloseValueTop = .5; // double that represents the position value to be applied to the openClose top servo
    double conveyorValueTop = .5; // double that represents the postion value to be applied to the left and right grabbers on top
    double stillOpenCloseValueTop = .745;  // double that represents the value of the top open close servo when it is still

    // Variable declarations

    //  declare touch seniors
    //TouchSensor touchSensorTop;
    //TouchSensor touchSensorBottom;

    // Bottom
    boolean dpadRightBottom = false; // boolean that is initially set to false, and becomes true if the dpad right is pressed
    boolean dpadLeftBottom = false; // boolean that is initially set to false, and becomes true if the dpad left is pressed
    double gamepadrightyBottom; // double that represents the raw value of the y axis of the right stick of the gamepad being used
    boolean rightBumperBottom = false; // boolean that is initially set to false, and becomes true if the right bumper is pressed
    double rightTriggerBottom; // double that represents the raw value of the right trigger of the gamepad being used
    double openCloseValueBottom = .5; // double that represents the position value to be applied to the openClose bottom servo
    double conveyorValueBottom = .5; // double that represents the postion value to be applied to the left and right grabbers on bottom
    double stillOpenCloseValue = .745;   // double that represents the value of the bottom open close servo when it is still

    // booleans for limit switch state
    //boolean touchTopPress = false;
    //boolean touchBottomPress = false;

    //doubles for adjustment constants
    double adjustedRightXTop;
    double adjustedLeftTriggerTop;

    double adjustedRightYBottom;
    double adjustedRightTriggerBottom;

    public void init ()
    {

        // harware map configurations
        //horizontalTop = hardwareMap.servo.get("horizontalTop");
        openCloseTop = hardwareMap.servo.get("openCloseTop");
        rightGrabberTop = hardwareMap.servo.get("rightGrabberTop");
        leftGrabberTop = hardwareMap.servo.get("leftGrabberTop");
        //horizontalBottom = hardwareMap.servo.get("horizontalBottom");
        openCloseBottom = hardwareMap.servo.get("openCloseBottom");
        leftGrabberBottom = hardwareMap.servo.get("leftGrabberBottom");
        rightGrabberBottom = hardwareMap.servo.get("rightGrabberBottom");


        //touchSensorTop = hardwareMap.touchSensor.get("touch1");
        //touchSensorBottom = hardwareMap.touchSensor.get("touch2");


        // Set servo direction orientations forward or reverse
        //horizontalTop.setDirection(Servo.Direction.FORWARD);
        openCloseTop.setDirection(Servo.Direction.FORWARD);
        rightGrabberTop.setDirection(Servo.Direction.REVERSE);
        leftGrabberTop.setDirection(Servo.Direction.FORWARD);

        //horizontalBottom.setDirection(Servo.Direction.FORWARD);
        openCloseBottom.setDirection(Servo.Direction.REVERSE);
        rightGrabberBottom.setDirection(Servo.Direction.FORWARD);
        leftGrabberBottom.setDirection(Servo.Direction.REVERSE);

        // Initial positions for servos
        //horizontalTop.setPosition(.486);
        openCloseTop.setPosition(.5);
        rightGrabberTop.setPosition(.5);
        leftGrabberTop.setPosition(.5);

        //horizontalBottom.setPosition(.486);
        openCloseBottom.setPosition(.5);
        rightGrabberBottom.setPosition(.5);
        leftGrabberBottom.setPosition(.5);
    } // end init

    public void loop() {

        // T O U C H   S E N S O R S
        // top touch sensor logic
        //if (touchSensorTop.isPressed()) {
        //    touchTopPress = true;
        //} else {
        //    touchTopPress = false;
        //}

        // bottom touch senor logic
        //if (touchSensorBottom.isPressed()) {
        //    touchBottomPress = true;
        //} else {
        //    touchBottomPress = false;
        //}


        // D P A D
        // dpad up logic
        if (gamepad1.dpad_up) {
            dpadUpTop = true;
        } else {
            dpadUpTop = false;
        }

        // dpad down logic
        if (gamepad1.dpad_down) {
            dpadDownTop = true;
        } else {
            dpadDownTop = false;
        }

        // dpad right logic
        if (gamepad1.dpad_right) {
            dpadRightBottom = true;
        } else {
            dpadRightBottom = false;
        }

        // dpad down logic
        if (gamepad1.dpad_left) {
            dpadLeftBottom = true;
        } else {
            dpadLeftBottom = false;
        }


        // B U M P E R S

        // left bumper logic
        if (gamepad1.left_bumper) {
            leftBumperTop = true;
        } else {
            leftBumperTop = false;
        }

        // right bumper logic
        if (gamepad1.right_bumper) {
            rightBumperBottom = true;
        } else {
            rightBumperBottom = false;
        }

        //Set gamepadright x and leftTriggger
        gamepadrightxTop = gamepad1.right_stick_x;
        leftTriggerTop = gamepad1.left_trigger;

        //Set gamepadright x and leftTriggger
        gamepadrightyBottom = gamepad1.right_stick_y;
        rightTriggerBottom = gamepad1.right_trigger;

        adjustedRightXTop = .486 - gamepadrightxTop / 2;
        adjustedLeftTriggerTop = leftTriggerTop / 2 + .5;

        adjustedRightYBottom = .486 - gamepadrightyBottom / 2;
        adjustedRightTriggerBottom = rightTriggerBottom / 2 + .5;


        //Adjust openCloseValue as necesarry

        if (dpadUpTop && !dpadDownTop) {
            openCloseValueTop += .012;
        }
        if (dpadDownTop && !dpadUpTop) {
            openCloseValueTop -= .012;
        }

        openCloseValueTop = Range.clip(openCloseValueTop, .157, .745);



        if (dpadRightBottom && !dpadLeftBottom) {
            openCloseValueBottom += .012;
        }
        if (dpadLeftBottom && !dpadRightBottom) {
            openCloseValueBottom -= .012;
        }

        openCloseValueBottom = Range.clip(openCloseValueBottom, .157, .745);




        if (leftTriggerTop > .05 && !leftBumperTop) {
            conveyorValueTop = adjustedLeftTriggerTop;
        } else if (leftTriggerTop < .05 && leftBumperTop) {
            conveyorValueTop = 0;
        }
        else if (leftTriggerTop < .05 && !leftBumperTop) {
            conveyorValueTop = .5;
        }
        else if (leftTriggerTop >.05 && leftBumperTop) {
            conveyorValueTop = .5;
        }
        else {
            conveyorValueTop = .5;
        }




        if (rightTriggerBottom > .05 && !rightBumperBottom) {
            conveyorValueBottom = adjustedRightTriggerBottom;
        } else if (rightTriggerBottom < .05 && rightBumperBottom) {
            conveyorValueBottom = 0;
        }
        else if (rightTriggerBottom < .05 && !rightBumperBottom) {
            conveyorValueTop = .5;
        }
        else if (rightTriggerBottom >.05 && rightBumperBottom) {
            conveyorValueTop = .5;
        }
        else {
            conveyorValueTop = .5;
        }


        //Set powers
        //horizontalTop.setPosition(adjustedRightXTop);
        //if (touchTopPress && conveyorValueTop > .5)
        //{
        //    conveyorValueTop = .5;
        //}

        rightGrabberTop.setPosition(conveyorValueTop);
        leftGrabberTop.setPosition(conveyorValueTop);
        //if (!touchTopPress)
        //{
            openCloseTop.setPosition(openCloseValueTop);
        //}
        //else
        //{
            //openCloseTop.setPosition(stillOpenCloseValue);
        //}



        //Set powers
        //horizontalBottom.setPosition(adjustedRightYBottom);
        //if (touchBottomPress && conveyorValueBottom > .5)
        //{
        //    conveyorValueBottom = .5;
        //}

        rightGrabberBottom.setPosition(conveyorValueBottom);
        leftGrabberBottom.setPosition(conveyorValueBottom);
        //if (!touchBottomPress)
        //{
            openCloseBottom.setPosition(openCloseValueBottom);
        //}
        //else
        //{
            //openCloseBottom.setPosition(stillOpenCloseValue);
        //}

    } // end loop

} // end class
