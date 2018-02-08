//TELEOP

//Package statement
package org.firstinspires.ftc.teamcode;

//Import statements
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import android.app.Activity;
import android.media.MediaPlayer;
import android.os.Bundle;
import android.os.Handler;
import android.view.View;

import android.widget.Button;
import android.widget.ImageView;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;
import java.util.concurrent.TimeUnit;

//Declaration for display on the driver station
@TeleOp(name = "TeleOpLimits", group = "LAMeets")

//Class declaration
public class TeleopLimits extends OpMode {

    //MOTORS

    //Driving
    DcMotor rightMotor; //Right drive motor, for driving forwards and backwards
    DcMotor leftMotor;  //Left drive motor, for driving forwards and backwards
    DcMotor backMotor;  //Back drive motor, for driving sideways, a.k.a "strafing"

    //Relic
    DcMotor relicMotor; //Motor to extend the relic scoring mechanism

    //Intake
    DcMotor rightIntakeMotor; //Motor that controls the right intake/right wheel of the intake
    DcMotor leftIntakeMotor; //Motor that controls the left intake/left wheel of the intake

    //Block Scoring
    DcMotor panLifterMotor; //Motor that lifts and lowers the block scoring mechanism, known as the "pan"

    //SERVOS

    //Block Rotation
    Servo leftPanSpin; //Servo on the left side of the robot that rotates the pan, or block scoring mechanism, in order to score blocks
    Servo rightPanSpin; //Servo on the right side of the robot that rotates the pan, or block scoring mechanism, in order to score blocks
    Servo upDown; //Servo to raise the relic mechanism up and down
    Servo grab; //Servo to grab the relic
    Servo panBack; //Servo to limit blocks from falling out of the robot
    Servo touchServo; //Servo that extends an arm from the front of the robot to detect when we are ready to score in autonomous
    //Feeler
    Servo feelerRaise; //Servo that lifts and lowers the ball scoring mechanism, known as the "feeler"
    double feelerRaiseUpPosition = .9; //Position that the feelerRaise is set to when we are not scoring the ball

    //Controls
    //Driving controls, all for gamepad 1
    double leftStickX1; //Value taken from the raw value of the left X stick on gamepad 1
    double rightStickX1; //Value taken from the raw value of the right X stick on gamepad 1
    double rightStickY1; //Value taken from the raw value of the right Y stick on gamepad 1

    //Cube movement controls, all for gamepad 1
    double rightTrigger; //double for the extent to which rightTrigger is pressed. No press is 0, full press is 1
    boolean rightBumper; //boolean for rightBumper. Set to true if rightBumper is pressed and set to false otherwise
    double leftTrigger; //double for the extent to which leftTrigger is pressed. No press is 0, full press is 1
    boolean leftBumper; //boolean for leftBumper. Set to true if leftBumper is pressed and set to false otherwise
    boolean dpadDownPressed; //boolean for dpadDown. Set to true if dpadDown is pressed and set to false otherwise
    boolean dpadUpPressed; //boolean for dpadUp. Set to true if dpadUp is pressed and set to false otherwise
    boolean dpadLeftPressed; //boolean for dpadLeft. Set to true if dpadLeft is pressed and set to false otherwise
    boolean yPressed; //boolean for the y button.  Set to true if y is pressed and set to false otherwise.
    boolean aPressed; //boolean for the a button.  Set to true if a is pressed and set to false otherwise.
    boolean xPressed; //boolean for the x button.  Set to true if x is pressed and set to false otherwise.
    boolean bPressed; //boolean for the b button.  Set to true if b is pressed and set to false otherwise.

    //Relic controls, all for gamepad 1
    boolean dpadRightPressed; //boolean for dpadRight. Set to true if dpadRight is pressed and set to false otherwise
    double grabPosition;
    double upDownPosition;

    //Driving variables
    double linLeftPower; //double that is set to leftStickX1
    double linRightPower; //double that is set to negative leftStickX1
    double linFrontPower; //double that is set to negative rightStickX1
    double linBackPower; //double that is set to rightStickX1
    double rotLeftPower; //double that is set to negative rightStickY1
    double rotRightPower; //double that is set to negative rightStickY1
    double testLeftPower; //double that is set to linLeftPower plus rotLeftPower
    double testRightPower; //double that is set to linRightPower plus rotRightPower
    double testBackPower; //double that is set to linBackPower
    double testFrontPower; //double that is set to linFrontPower
    int leftOverOne; //double that is set to 1 if the absolute value of testLeftPower is greater than 1, and is set to 0 otherwise
    int rightOverOne; //double that is set to 1 if the absolute value of testRightPower is greater than 1, and is set to 0 otherwise
    int frontOverOne; //double that is set to 1 if the absolute value of testFrontPower is greater than 1, and is set to 0 otherwise
    int backOverOne; //double that is set to 1 if the absolute value of testBackPower is greater than 1, and is set to 0 otherwise
    boolean single = true; //boolean that is set to true if the sum of leftOverOne, rightOverOne, frontOverOne, and backOverOne is equal to or greater than 1, and is set to false otherwise
    int direction = 0; //int that is set to either 0, 1, 2, 3, 4 depending on the values of leftOverOne, rightOverOne, frontOverOne, and backOverOne
    int over; //int that is set to 1 if the sum of leftOverOne, rightOverOne, frontOverOne, and backOverOne is equal to or greater than 1, and is set to 0 otherwise
    double finLeftPower; //The final power to be applied to leftMotor
    double finRightPower; //The final power to be applied to rightMotor
    double finFrontPower; //The final power to be applied to backMotor
    double finBackPower; //The final power to be applied to backMotor

    //Block variables
    double leftIntakePower; //The power to which we set leftIntakeMotor
    double rightIntakePower; //The power to which we set rightIntakeMotor
    double panLiftingPower; //The power to which we set panLifterMotor
    double panSpinPosition; //The position to which we set leftPanSpin and rightPanSpin

    //BLOCK SENSORS

    DistanceSensor sensorA; //Distance sensor closest to the intake to see how far away potential blocks are
    DistanceSensor sensorB; //Distance sensor between A and C, to see how far potential blocks are
    DistanceSensor sensorC; //Distance sensor farthest from intake, to see how far potential blocks are

    //Limit Switches
    DigitalChannel limitTop; //Limit Switch that tells us if we reach the top of the robot with the Pan
    DigitalChannel limitBottom; //Limit switch that tells us if we reach the bottom of the robot with the Pan

    // Gain Mode Booleans
    boolean highGain; //boolean that is true if we are making driving faster and is false otherwise
    boolean lowGain; //boolean that is true if we are making driving slower and is false otherwise
    boolean gainToggle; //boolean that is true if the joystick right stick button and is false otherwise

    //Mode booleans
    boolean previousStatus; //Boolean that is true if the previous mode was end game, and is false otherwise
    boolean currentStatus; //Boolean that is true if the current mode is endgame, and is false otherwise

    /* Initialize standard Hardware interfaces */
    public void init() { // use hardwaremap here instead of hwmap or ahwmap provided in sample code

        //Motor configurations in the hardware map
        rightMotor = hardwareMap.dcMotor.get("m1"); //m1
        leftMotor = hardwareMap.dcMotor.get("m2"); //m2
        backMotor = hardwareMap.dcMotor.get("m3"); //m3
        rightIntakeMotor = hardwareMap.dcMotor.get("m4"); //m4
        leftIntakeMotor = hardwareMap.dcMotor.get("m5"); //m5
        panLifterMotor = hardwareMap.dcMotor.get("m6"); //m6
        relicMotor = hardwareMap.dcMotor.get("m7"); //m7

        //Servo configurations in the hardware map
        leftPanSpin = hardwareMap.servo.get("s1"); //s1
        rightPanSpin = hardwareMap.servo.get("s2"); //s2
        grab = hardwareMap.servo.get("s6"); //s6
        feelerRaise = hardwareMap.servo.get("s8"); //s8
        upDown = hardwareMap.servo.get("s11"); //s11
        panBack = hardwareMap.servo.get("s12"); //s12
        touchServo = hardwareMap.servo.get("s10"); //s10

        //Code to extend the upDown Servo past 180 degrees
        ServoControllerEx theControl = (ServoControllerEx) upDown.getController();
        int thePort = upDown.getPortNumber();
        PwmControl.PwmRange theRange = new PwmControl.PwmRange(453, 2600);
        theControl.setServoPwmRange(thePort, theRange);

        limitTop = hardwareMap.get(DigitalChannel.class, "d1"); //d1
        limitBottom = hardwareMap.get(DigitalChannel.class, "d2"); //d2
        sensorA = hardwareMap.get(DistanceSensor.class, "i2"); //i2
        sensorB = hardwareMap.get(DistanceSensor.class, "i3"); //i3
        sensorC = hardwareMap.get(DistanceSensor.class, "i4"); //i4

        // Motor directions
        rightMotor.setDirection(REVERSE); //Set rightMotor to REVERSE direction
        leftMotor.setDirection(FORWARD); //Set leftMotor to FORWARD direction
        backMotor.setDirection(REVERSE); //Set backMotor to REVERSE direction
        rightIntakeMotor.setDirection(REVERSE); //Set rightIntakeMotor to REVERSE direction
        leftIntakeMotor.setDirection(FORWARD); //Set leftIntakeMotor to FORWARD direction
        panLifterMotor.setDirection(REVERSE); //Set panLifterMotor to REVERSE direction
        relicMotor.setDirection(REVERSE); //Set relicMotor to REVERSE direction

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Set rightMotor mode to BRAKE
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Set leftMotor mode to BRAKE
        backMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Set backMotor mode to BRAKE

        // Servo directions
        leftPanSpin.setDirection(Servo.Direction.REVERSE); //Set leftPanSpin to REVERSE direction
        rightPanSpin.setDirection(Servo.Direction.FORWARD); //Set rightPanSpin to FORWARD direction
        grab.setDirection(Servo.Direction.REVERSE); //Set grab to REVERSE direction
        upDown.setDirection(Servo.Direction.FORWARD); //Set upDown to FORWARD direction
        feelerRaise.setDirection(Servo.Direction.FORWARD); //Set feelerRaise to FORWARD direction
        panBack.setDirection(Servo.Direction.FORWARD); //Set panBack to FORWARD direction
        touchServo.setDirection(Servo.Direction.REVERSE); //Set touchServo to REVERSE direction

        //Init values
        leftPanSpin.setPosition(.2175); //Set leftPanSpin to position .1875
        rightPanSpin.setPosition(.2375); //Set rightPanSpin to position .2075
        grab.setPosition(.375); //Set grab to position .375
        upDown.setPosition(0); //Set upDown to position 0
        panBack.setPosition(0.39); //Set panBack to position .39
        feelerRaise.setPosition(feelerRaiseUpPosition); //Set feelerRaise to feelerRaiseUpPosition
        touchServo.setPosition(.65); //Set touchServo to position .65

        highGain = true;
        lowGain = false;
        gainToggle = true;
    }
    public void loop () {

        dpadRightPressed = gamepad1.dpad_right; //Set variable dpadRightPressed to the raw boolean value of the right dpad

        if(dpadRightPressed) //If dpadRightPressed is true
        {
            if (!previousStatus) //If previousStatus is false
            {
                currentStatus = true; //Set currentStatus to true
            }
            else //Else
            {
                currentStatus = false; //Set currentStatus to false
            }
        }
        else //Else
        {
            previousStatus = currentStatus; //Set previousStatus to currentStatus
        }

        //Telemetry
        telemetry.addData("dpad right", dpadRightPressed);
        telemetry.addData("current", currentStatus);

        //DRIVING
        telemetry.addData("Left X Joy Raw: ", gamepad1.left_stick_x); //The raw value of left stick x
        telemetry.addData("Right X Joy Raw: ", gamepad1.right_stick_x); //The raw value of right stick x
        telemetry.addData("Right Y Joy Raw: ", gamepad1.right_stick_y); //The raw value of right stick y

        //Set leftStickX1 to the left stick x value of gamepad 1 times the absolute value of this left stick x value
        leftStickX1 = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);
        //Set rightStickX1 to the right stick x value of gamepad 1 times the absolute value of this right stick x value
        rightStickX1 = gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x);
        //Set rightStickY1 to the right stick y value of gamepad 1 times the absolute value of this right stick y value
        rightStickY1 = gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y);

        //Set driving variables to values specified in the variable declaration section
        linLeftPower = leftStickX1;
        linRightPower = -leftStickX1;
        linFrontPower = -rightStickX1;
        linBackPower = rightStickX1;
        rotLeftPower = -rightStickY1;
        rotRightPower = -rightStickY1;
        testLeftPower = linLeftPower + rotLeftPower;
        testRightPower = linRightPower + rotRightPower;
        testBackPower = linBackPower;
        testFrontPower = linFrontPower;

        feelerRaise.setPosition(feelerRaiseUpPosition); //Set feelerRaise to feelerRaiseUpPosition

        if (Math.abs(testLeftPower) > 1) //Tests if each Math.abs(testLeftPower) is over 1
        {
            leftOverOne = 1;
            direction = 1;
        } else {
            leftOverOne = 0;
        }

        if (Math.abs(testRightPower) > 1) { //Tests if each Math.abs(testRightPower) is over 1
            rightOverOne = 1;
            direction = 2;
        } else {
            rightOverOne = 0;
        }

        if (Math.abs(testFrontPower) > 1) { //Tests if each Math.abs(testFrontPower) is over 1
            frontOverOne = 1;
            direction = 3;
        } else {
            frontOverOne = 0;
        }

        if (Math.abs(testBackPower) > 1) { //Tests if each Math.abs(testBackPower) is over 1
            backOverOne = 1;
            direction = 4;
        } else {
            backOverOne = 0;
        }

        if ((leftOverOne + rightOverOne + frontOverOne + backOverOne) > 1) //Test if sum of "OverOne" values is over 1
        {
            single = false;
            over = 1;
        } else if ((leftOverOne + rightOverOne + frontOverOne + backOverOne) == 1) {
            single = true;
            over = 1;
        } else {
            over = 0;
        }


        switch (over) { //Switch statement with int over
            case 0: //if over equals 1
                //Set all fin powers to their respective test powers
                finLeftPower = testLeftPower;
                finRightPower = testRightPower;
                finFrontPower = testFrontPower;
                finBackPower = testBackPower;
                break; //End case 0
            case 1:
                if (single == true) { //If single is true
                    switch (direction) { //Switch statement with int direction
                        case 1: //If direction equals 1
                            //Set all fin powers to their respective test powers divided by the absolute values of their respective test powers
                            finLeftPower = testLeftPower / Math.abs(testLeftPower);
                            finRightPower = testRightPower / Math.abs(testLeftPower);
                            finFrontPower = testFrontPower / Math.abs(testLeftPower);
                            finBackPower = testBackPower / Math.abs(testLeftPower);
                            break; //End case 1
                        case 2: //If direction equals 2
                            finLeftPower = testLeftPower / Math.abs(testRightPower); //Set finLeftPower to testLeftPower divided by the absolute value of testRightPower
                            finRightPower = testRightPower / Math.abs(testRightPower); //Set finRightPower to testRightPower divided by the absolute value of testRightPower
                            finFrontPower = testFrontPower / Math.abs(testRightPower); //Set finFrontPower to testFrontPower divided by the absolute value of testRightPower
                            finBackPower = testBackPower / Math.abs(testRightPower); //Set finBackPower to testBackPower divided by the absolute value of testRightPower
                            break; //End case 2
                        case 3: //If direction equals 3
                            finLeftPower = testLeftPower / Math.abs(testFrontPower); //Set finLeftPower to testLeftPower divided by the absolute value of testFrontPower
                            finRightPower = testRightPower / Math.abs(testFrontPower); //Set finRightPower to testRightPower divided by the absolute value of testFrontPower
                            finFrontPower = testFrontPower / Math.abs(testFrontPower); //Set finFrontPower to testLeftPower divided by the absolute value of testFrontPower
                            finBackPower = testBackPower / Math.abs(testFrontPower); //Set finBackPower to testBackPower divided by the absolute value of testFrontPower
                            break; //End case 3
                        case 4: //If direction equals 4
                            finLeftPower = testLeftPower / Math.abs(testBackPower); //Set finLeftPower to testLeftPower divided by the absolute value of testBackPower
                            finRightPower = testRightPower / Math.abs(testBackPower); ////Set finRightPower to testRightPower divided by the absolute value of testBackPowerSet finRightPower to testRightPower divided by the absolute value of testBackPower
                            finFrontPower = testFrontPower / Math.abs(testBackPower);  //Set finFrontPower to testFrontPower divided by the absolute value of testBackPower
                            finBackPower = testBackPower / Math.abs(testBackPower); //Set finBackPower to testBackPower divided by the absolute value of testBackPower
                            break; //End case 4
                        default: //If direction is not any of the options above
                            telemetry.addData("Return", "Less than 1"); //Telemetry signaling default case is active
                            break; //End default case
                    }
                } else { //If single is not true
                    //Set maxPower to the maximum value of the absolute values of all test values
                    double maxPower = Math.max(Math.max(Math.abs(testLeftPower), Math.abs(testRightPower)), Math.max(Math.abs(testFrontPower), Math.abs(testBackPower)));
                    finLeftPower = testLeftPower / maxPower; //Set finLeftPower to testLeftPower divided by maxPower
                    finRightPower = testRightPower / maxPower; //Set finRightPower to finRightPower divided by maxPower
                    finFrontPower = testFrontPower / maxPower; //Set finFrontPower to testFrontPower divided by maxPower
                    finBackPower = testBackPower / maxPower; //Set finBackPower to testBackPower divided by maxPower
                }
                break; //End case 1
            default: //If single neither true or not true
                telemetry.addData("Something", "Messed up"); //Telemetry signaling error in execution
                break; //End default case
        }

        //Squared driving
        if (finBackPower<-.01) //If finBackPower is adequately negative
        {
            finBackPower= -1* finBackPower * finBackPower; //Set finBackPower to -1 times finBackPower squared
        }
        if (finBackPower>.01) //If finBackPower is adequately positive
        {
            finBackPower= finBackPower* finBackPower; //Set finBackPower to finBackPower squared
        }

        if (finLeftPower<-.01) //If finLeftPower is adequately negative
        {
            finLeftPower= -1* finLeftPower * finLeftPower; //Set finLeftPower to -1 times finLeftPower squared
        }
        if (finLeftPower>.01) //If finLeftPower is adequately positive
        {
            finLeftPower= finLeftPower* finLeftPower; //Set finLeftPower to finLeftPower squared
        }

        if (finRightPower<-.01) //If finRightPower is adequately negative
        {
            finRightPower= -1* finRightPower * finRightPower; //Set finRightPower to -1 times finRightPower squared
        }
        if (finRightPower>.01) //If finRightPower is adequately positive
        {
            finRightPower= finRightPower* finRightPower; //Set finRightPower to finRightPower squared
        }


        //Code to increase strafing power
        if (Math.abs(gamepad1.right_stick_x) > 0.05) //If we are strafing at an adequate power
        {
            finLeftPower = finLeftPower+ .3*gamepad1.right_stick_x; //Add .3 times gamepad1.right_stick_x to finLeftPower
            finRightPower = finRightPower - .3*gamepad1.right_stick_x; //Subtract .3 times gamepad1.right_stick_x from finRightPower

        }

        //Code to decrease leftMotor and rightMotor power
        if (Math.abs(gamepad1.left_stick_x) > 0.05) //If we are using the left and right motors at adequate power
        {
            finLeftPower /= 1.6; //Divide finLeftPower by 1.6
            finRightPower /= 1.6; //Divide finRightPower by 1.6
        }

        //Clip final driving motor values between -1 and 1
        finBackPower = Range.clip(finBackPower, -1, 1);
        finRightPower = Range.clip(finRightPower, -1, 1);
        finLeftPower = Range.clip(finLeftPower, -1, 1);

        //TRANSPORTING
        //Controller Inputs

        rightTrigger = gamepad1.right_trigger; //Set variable rightTrigger to the raw double value of the right trigger
        rightBumper = gamepad1.right_bumper; //Set variable rightBumper to the raw boolean value of the right bumper
        leftTrigger = gamepad1.left_trigger; //Set variable leftTrigger to the raw double value of the left trigger
        leftBumper = gamepad1.left_bumper; //Set variable leftBumper to the raw boolean value of the left bumper
        dpadLeftPressed = gamepad1.dpad_left; //Set variable dpadLeftPressed to the raw boolean value of the left dpad
        dpadUpPressed = gamepad1.dpad_up; //Set variable dpadUpPressed to the raw boolean value of the up dpad
        dpadDownPressed = gamepad1.dpad_down; //Set variable dpadDownPressed to the raw boolean value of the down dpad
        aPressed =gamepad1.a; //Set variable aPressed to the raw boolean value of the a button
        yPressed = gamepad1.y; //Set variable yPressed to the raw boolean value of the y button


        if (!currentStatus) { //If currentStatus is false

            if (rightTrigger > .05 && leftBumper) { //If both rightTrigger and leftBumper are pressed, set intake powers to 0, as this is a conflicting command
                leftIntakePower = 0; //Set leftIntakePower to 0
                rightIntakePower = 0; //Set rightIntakePower to 0
            } else if (rightTrigger > .05) { //Else if rightTrigger if pressed
                leftIntakePower = Math.pow(rightTrigger, 2) * .75; //Set leftIntakePower to the square of rightTrigger times .6
                rightIntakePower = Math.pow(rightTrigger, 2) * .85; //Set rightIntakePower to the square of rightTrigger times .8
            } else if (leftBumper) { //Else if leftBumper is pressed
                leftIntakePower = -.7; //Set leftIntakePower to -.7
                rightIntakePower = -.7; //Set rightIntakePower to -.7
            } else { //Else
                leftIntakePower = 0; //Set leftIntakePower to 0
                rightIntakePower = 0; //Set rightIntakePower to 0
            }

            if (dpadDownPressed && dpadUpPressed) //If dpadUp and dpadDown are pressed
            {
                panLiftingPower = 0; //Set panLiftingPower to 0 due to conflicting commands
            }

            else if (dpadUpPressed) //Else if dpadUp is pressed
            {
                panLiftingPower = -1; //Set panLiftingPower to -1
            }

            else if (dpadDownPressed)  //Else if dpadDown is pressed
            {
                panLiftingPower = 1; //Set panLiftingPower to 1
            }

            else //Else
            {
                panLiftingPower = 0; //Set panLiftingPower to 0
            }

//            if (dpadLeftPressed) {
//                panBack.setPosition(0.45); //Set panBack to position .45
//            }

            yPressed = gamepad1.y; //Set boolean yPressed to gamepad1.y
            aPressed = gamepad1.a; //Set boolean aPressed to gamepad1.a
            xPressed = gamepad1.x; //Set boolean xPressed to gamepad1.x
            bPressed = gamepad1.b; //Set boolean bPressed to gamepad1.b

            if (dpadLeftPressed) //if dpad left is pressed
            {
                panBack.setPosition(.39); //Set panBack to position .39
            }

            if (yPressed && aPressed) //If y and a are pressed
            {
                //Do nothing due to conflicting commands
            }
            else if (yPressed) //Else if y is pressed
            {
                panSpinPosition = panSpinPosition + .05; //Add .05 to panSpinPosition
                panBack.setPosition(.9); //Set panBack to position .9
            }

            else if (aPressed) //Else if a is pressed
            {
                panSpinPosition = panSpinPosition - .05; //Subtract .05 to panSpinPosition
            }

            else if (xPressed) //Else if x is pressed
            {
                panSpinPosition = .3; //Set panSpinPosition to .3
                panBack.setPosition(0.39); //Set panBack to position .39
            }

            else if (bPressed) //Else if b is pressed
            {
                panSpinPosition = .1875; //Set panSpinPosition to .1875
                panBack.setPosition(0.39); //Set panBack to position .39
            }

            //Telemetry
            telemetry.addData("panLifting", panLiftingPower);
            telemetry.addData("limitTop", limitTop.getState());
            telemetry.addData("limitBottom", limitBottom.getState());

            panSpinPosition = Range.clip(panSpinPosition, .205, .625); //Ensure panSpinPosition is between .205 and .625
        }

        //Else
        else
        {

            if (dpadUpPressed) //If dPadUpPressed is true
            {
                relicMotor.setPower(.6); //Set the power of relic motor to .6
            }
            else if (dpadDownPressed) //Else if dpadDownPressed is true
            {
                relicMotor.setPower(-.6); //Set the power of relic motor to -.6
            }
            else
            {
                relicMotor.setPower(0); //Set the power of relic motor to 0
            }

            if (rightTrigger > .05  && leftTrigger > .05) //If rightTrigger and leftTrigger are adequately pressed
            {
                //Do nothing due to conflicting commands
            }
            else if (leftTrigger>.05) //Else if leftTrigger is adequately pressed
            {
                grabPosition+=.01 * leftTrigger; //Add .01 times the value of leftTrigger to grabPosition
            }
            else if (rightTrigger>.05)
            {
                grabPosition-=.01 * rightTrigger; //Subtract .01 times the value of leftTrigger to grabPosition
            }
            else //Else
            {
                //Do nothing
            }

            if (yPressed && aPressed) //If yPressed and aPressed are true
            {
                //Do nothing due to conflicting commands
            }
            else if (aPressed) //Else if aPressed is true
            {
                upDownPosition+=.003; //Add .003 to upDownPosition
            }
            else if (yPressed) //Else if yPressed is true
            {
                upDownPosition-=.003; //Subract .003 from upDownPosition
            }
            else //Else
            {
                //Do nothing
            }

            grabPosition=Range.clip(grabPosition, .045, .375); //Ensure grabPosition is between .045 and .375
            upDownPosition=Range.clip(upDownPosition, 0, .83); //Ensure upDownPosition is between 0 and 1
        }

        if (gamepad1.right_stick_button && highGain && gainToggle)
        {
            lowGain = true;
            highGain = false;
            gainToggle = false;
        }
        else if (gamepad1.right_stick_button && lowGain && gainToggle)
        {
            highGain = true;
            lowGain = false;
            gainToggle = false;
        }
        else if ( !gamepad1.right_stick_button )
        {
            gainToggle = true;
        }

        if (lowGain)
        {
            finLeftPower = finLeftPower*.5;
            finRightPower = finRightPower*.5;
        }

        //SET CONTROLS
        leftPanSpin.setPosition(panSpinPosition); //Set the position of leftPanSpin to panSpinPosition
        rightPanSpin.setPosition(panSpinPosition + .02); //Set the position of rightPanSpin to panSpinPosition plus .02
        panLifterMotor.setPower(panLiftingPower); //Set the power of panLifterMotor to panLiftingPower
        leftIntakeMotor.setPower(leftIntakePower); //Set the power of leftIntakeMotor to leftIntakePower
        rightIntakeMotor.setPower(rightIntakePower); //Set the power of rightIntakeMotor to rightIntakePower
        leftMotor.setPower(finLeftPower); //Set the power of leftMotor to finLeftPower
        rightMotor.setPower(finRightPower); //Set the power of rightMotor to finRightPower
        backMotor.setPower(finBackPower); //Set the power of backMotor to finBackPower
        grab.setPosition(grabPosition); //Set the power of grab to grabPosition
        upDown.setPosition(upDownPosition); //Set the power of upDown to upDownPosition

        //Update telemetry
        telemetry.update();
    } // end loop

} // End class
