
/**
 * TeleopV2 Made Saturday December 9 by Isaac, Marcus, and Steve
 */
//Import Statements

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
import java.util.concurrent.TimeUnit;

@TeleOp(name = "TeleOp1Mod", group = "LAMeets")
public class Teleop1Mod extends OpMode {

    //Controls and Variables
    //Driving controls
    double leftStickX1; //Adjusted value taken from the raw value of the left X stick on gamepad 1
    double rightStickX1; //Adjusted value taken from the raw value of the right X stick on gamepad 1
    double rightStickY1; //Adjusted value taken from the raw value of the right Y stick on gamepad 1

    //Driving variables
    boolean single = true;
    int leftOverOne;
    int rightOverOne;
    int frontOverOne;
    int backOverOne;
    int direction;
    int over;
    int mode = 0; //end game or normal mode, init to normal
    int longArmAngle;
    int shortArmAngle;
    double linLeftPower;
    double linRightPower;
    double linFrontPower;
    double linBackPower;
    double rotLeftPower;
    double rotRightPower;
    double rotBackPower;
    double rotFrontPower;
    double testLeftPower;
    double testRightPower;
    double testBackPower;
    double testFrontPower;
    double finLeftPower;
    double finRightPower;
    double finFrontPower;
    double finBackPower;

    //Driving Dc Motors
    DcMotor rightMotor;
    DcMotor leftMotor;
    DcMotor backRightMotor;
    //DcMotor backLeftMotor;

    //Cube movement variables
    double rightTrigger; //double for the extent to which rightTrigger is pressed. No press is 0, full press is 1
    boolean rightBumper; //boolean for rightBumper. Set to true if rightBumper is pressed and set to false otherwise
    double leftTrigger; //double for the extent to which leftTrigger is pressed. No press is 0, full press is 1
    boolean leftBumper; //boolean for leftBumper. Set to true if leftBumper is pressed and set to false otherwise
    double panSpinPosition; //boolean that represents whether the pan is in intake or scoring position
    boolean dpadDownPressed; //boolean for dpadDown. Set to true if dpadDown is pressed and set to false otherwise
    boolean dpadRightPressed; //boolean for dpadRight. Set to true if dpadRight is pressed and set to false otherwise
    boolean dpadLeftPressed; //boolean for dpadLeft. Set to true if dpadLeft is pressed and set to false otherwise
    boolean dpadUpPressed; //boolean for dpadUp. Set to true if dpadUp is pressed and set to false otherwise
    boolean backPressed;
    boolean startPressed;
    double leftIntakePower;
    double rightIntakePower;
    double panLiftingPower;
    double leftStickY1;

    // Motor and servo configurations
    DcMotor rightIntakeMotor; //Dc motor that controls the right intake/right wheel of the intake
    DcMotor leftIntakeMotor; //Dc motor that controls the left intake/left wheel of the intake
    DcMotor panLifterMotor;
    DcMotor relicMotor;
    Servo leftPanSpin;
    Servo rightPanSpin;
    Servo panKick;
    Servo relicLongArm;
    Servo relicShortArm;
    Servo grab;
    Servo relicRotate;
    Servo extend;
    Servo swipe;
    double grabPosition = 0;
    double relicLongArmPosition;
    double relicShortArmPosition;
    double relicRotatePosition = 0;

    DigitalChannel limitTop; //Touch sensor that tells us if we reach the top with the Pan
    DigitalChannel limitBottom; //Touch sensor that tells us if we reach the bottom with the Pan

    boolean endGame;
    boolean previousStatus;
    boolean currentStatus;
    boolean yPressed;
    boolean aPressed;
    boolean xPressed;

    /* Initialize standard Hardware interfaces */
    public void init() { // use hardwaremap here instead of hwmap or ahwmap provided in sample code

        // Motor configurations in the hardware map
        rightMotor = hardwareMap.dcMotor.get("m1"); //RightMotor configuration
        leftMotor = hardwareMap.dcMotor.get("m2"); //LeftMotor configuration
        //backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor"); //BackLeftMotor configuration
        backRightMotor = hardwareMap.dcMotor.get("m3"); //BackRightMotor configuration
        rightIntakeMotor = hardwareMap.dcMotor.get("m4"); //rightIntakeMotor configuration
        leftIntakeMotor = hardwareMap.dcMotor.get("m5"); //leftIntakeMotor configuration
        panLifterMotor = hardwareMap.dcMotor.get("m6"); //panLifterMotor configuration
        //relicMotor = hardwareMap.dcMotor.get("m7");

        // Servo configurations in the hardware map
        leftPanSpin = hardwareMap.servo.get("s1"); //leftPanSpin configuration
        rightPanSpin = hardwareMap.servo.get("s2"); //rightPanSpin configuration
        //relicLongArm = hardwareMap.servo.get("s3");
        //relicShortArm = hardwareMap.servo.get("s4");
        //panKick = hardwareMap.servo.get("s5");
        //grab = hardwareMap.servo.get("s6");
        //relicRotate = hardwareMap.servo.get("s7");

        extend = hardwareMap.servo.get("s8");
        swipe = hardwareMap.servo.get("s9");

        // Touch sensor configuration in the hardware map
        //limitTop = hardwareMap.get(DigitalChannel.class, "d1"); //Top touchSensor configuration
        //limitBottom = hardwareMap.get(DigitalChannel.class, "d2");

        // Motor directions
        rightMotor.setDirection(REVERSE); //Set rightMotor to FORWARD direction
        leftMotor.setDirection(FORWARD); //Set leftMotor to REVERSE direction
        backRightMotor.setDirection(REVERSE); //Set backRightMotor to REVERSE direcion
        //backLeftMotor.setDirection(REVERSE); //Set backLeftMotor to REVERSE direction
        rightIntakeMotor.setDirection(REVERSE); //Set rightIntakeMotor to FORWARD direction
        leftIntakeMotor.setDirection(FORWARD); //Set leftIntakeMotor to FORWARD direction
        panLifterMotor.setDirection(REVERSE); //Set panLifterMotor to FORWARD direction
        //relicMotor.setDirection(FORWARD);

        // Servo directions
        leftPanSpin.setDirection(Servo.Direction.REVERSE); //Set leftPanSpin to REVERSE direction
        rightPanSpin.setDirection(Servo.Direction.FORWARD); //Set rightPanSpin to FORWARD direction
        swipe.setDirection(Servo.Direction.REVERSE);
        //panKick.setDirection(Servo.Direction.FORWARD);
        //grab.setDirection(Servo.Direction.REVERSE);
        //relicRotate.setDirection(Servo.Direction.REVERSE);

        //Init powers
        rightMotor.setPower(0); //Set rightMotor to 0 power
        leftMotor.setPower(0); //Set leftMotor to 0 power
        backRightMotor.setPower(0); //Set backRightMotor to 0 power
        //backLeftMotor.setPower(0); //Set backLeftMotor to 0 power
        rightIntakeMotor.setPower(0); //Set rightIntakeMotor to 0 power
        leftIntakeMotor.setPower(0); //Set leftIntakeMotor to 0 power
        panLifterMotor.setPower(0); //Set panLifterMotor to 0 power

        //grab.setPosition(.2);
        //relicRotate.setPosition( .3 );

        leftPanSpin.setPosition(.1);
        rightPanSpin.setPosition(.1);
        swipe.setPosition(0);
        extend.setPosition(.5);


        //panKick.setPosition(1);

        //endGame = false;
        //previousStatus = false;
        //currentStatus = false;
    }
    public void loop () {

        //DRIVING

        telemetry.addData("Left X Joy Raw: ", gamepad1.left_stick_x); //The raw value of left stick x
        telemetry.addData("Right X Joy Raw: ", gamepad1.right_stick_x); //The raw value of right stick x
        telemetry.addData("Right Y Joy Raw: ", gamepad1.right_stick_y); //The raw value of right stick y

        leftStickX1 = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);
        leftStickY1 = gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y);
        //Set rightStickX1 to the left stick x value of gamepad 1 times the absolute value of this left stick x value
        rightStickX1 = gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x);
        //Set rightStickX1 to the right stick x value of gamepad 1 times the absolute value of this right stick x value
        rightStickY1 = gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y);
        //Set rightStickY1 to the right stick x value of gamepad 1 times the absolute value of this right stick y value

        //startPressed = gamepad1.start;
        //startPressed = gamepad1.start;

        //Driving

        telemetry.addData("Left X Joy Raw: ", gamepad1.left_stick_x); //The raw value of left stick x
        telemetry.addData("Right X Joy Raw: ", gamepad1.right_stick_x); //The raw value of right stick x
        telemetry.addData("Right Y Joy Raw: ", gamepad1.right_stick_y); //The raw value of right stick y

        leftStickX1 = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);
        //Set rightStickX1 to the left stick x value of gamepad 1 times the absolute value of this left stick x value
        rightStickX1 = gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x);
        //Set rightStickX1 to the right stick x value of gamepad 1 times the absolute value of this right stick x value
        rightStickY1 = gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y);
        //Set rightStickY1 to the right stick x value of gamepad 1 times the absolute value of this right stick y value

        startPressed = gamepad1.start;
        //startPressed = gamepad1.start;

        if(startPressed)
        {
            if (!previousStatus)
            {
                currentStatus = true;
            }
            else
            {
                currentStatus = false;
            }
        }
        else
        {
            previousStatus = currentStatus;
        }

        linLeftPower = leftStickX1;
        linRightPower = -leftStickX1;
        linFrontPower = -rightStickX1;
        linBackPower = rightStickX1;
        rotLeftPower = -rightStickY1;
        rotRightPower = -rightStickY1;

        testLeftPower = linLeftPower + rotLeftPower;
        testRightPower = linRightPower + rotRightPower;
        testBackPower = linBackPower + rotBackPower;
        testFrontPower = linFrontPower + rotFrontPower;

        if (Math.abs(testLeftPower) > 1) //Tests if each Math.abs(testDirectionPower) is over 1 or not if it is, set respective bool to true
        {
            leftOverOne = 1;
            direction = 1;
        } else {
            leftOverOne = 0;
        }

        if (Math.abs(testRightPower) > 1) {
            rightOverOne = 1;
            direction = 2;
        } else {
            rightOverOne = 0;
        }

        if (Math.abs(testFrontPower) > 1) {
            frontOverOne = 1;
            direction = 3;
        } else {
            frontOverOne = 0;
        }

        if (Math.abs(testBackPower) > 1) {
            backOverOne = 1;
            direction = 4;
        } else {
            backOverOne = 0;
        }

        if ((leftOverOne + rightOverOne + frontOverOne + backOverOne) > 1) //Test if more than one testDirectionPower is over 1 i true, set single to false
        {
            single = false;
            over = 1;
        } else if ((leftOverOne + rightOverOne + frontOverOne + backOverOne) == 1) {
            single = true;
            over = 1;
        } else {
            over = 0;
        }

        switch (over) {
            case 0:
                finLeftPower = testLeftPower;
                finRightPower = testRightPower;
                finFrontPower = testFrontPower;
                finBackPower = testBackPower;
                break;
            case 1:
                if (single == true) {
                    switch (direction) {
                        case 1:
                            finLeftPower = testLeftPower / Math.abs(testLeftPower);
                            finRightPower = testRightPower / Math.abs(testLeftPower);
                            finFrontPower = testFrontPower / Math.abs(testLeftPower);
                            finBackPower = testBackPower / Math.abs(testLeftPower);
                            break;
                        case 2:
                            finLeftPower = testLeftPower / Math.abs(testRightPower);
                            finRightPower = testRightPower / Math.abs(testRightPower);
                            finFrontPower = testFrontPower / Math.abs(testRightPower);
                            finBackPower = testBackPower / Math.abs(testRightPower);
                            break;
                        case 3:
                            finLeftPower = testLeftPower / Math.abs(testFrontPower);
                            finRightPower = testRightPower / Math.abs(testFrontPower);
                            finFrontPower = testFrontPower / Math.abs(testFrontPower);
                            finBackPower = testBackPower / Math.abs(testFrontPower);
                            break;
                        case 4:
                            finLeftPower = testLeftPower / Math.abs(testBackPower);
                            finRightPower = testRightPower / Math.abs(testBackPower);
                            finFrontPower = testFrontPower / Math.abs(testBackPower);
                            finBackPower = testBackPower / Math.abs(testBackPower);
                            break;
                        default:
                            telemetry.addData("Return", "Less than 1");
                            break;
                    }
                } else {
                    double maxPower = Math.max(Math.max(Math.abs(testLeftPower), Math.abs(testRightPower)), Math.max(Math.abs(testFrontPower), Math.abs(testBackPower)));
                    finLeftPower = Math.pow(testLeftPower / maxPower,2);
                    finRightPower = Math.pow(testRightPower / maxPower,2);
                    finFrontPower = Math.pow(testFrontPower / maxPower,2);
                    finBackPower = Math.pow(testBackPower / maxPower,2);
                }
                break;
            default:
                telemetry.addData("Something", "Messed up");
                break;
        }

        finBackPower = Range.clip(finBackPower, -1, 1);
        finRightPower = Range.clip(finRightPower, -1, 1);
        finLeftPower = Range.clip(finLeftPower, -1, 1);

        //if ((finBackPower<0&&finRightPower>0)||(finBackPower>0&&finRightPower<0))
        //{
        //finRightPower/=4;
        //finLeftPower/=4;
        //}


        //TRANSPORTING
        //Controller Inputs

        rightTrigger = gamepad1.right_trigger; //Set variable rightTrigger to the raw double value of the right trigger
        rightBumper = gamepad1.right_bumper; //Set variable rightBumper to the raw boolean value of the right bumper
        leftTrigger = gamepad1.left_trigger;
        leftBumper = gamepad1.left_bumper;
        dpadLeftPressed = gamepad1.dpad_left;
        dpadRightPressed = gamepad1.dpad_right;
        dpadUpPressed = gamepad1.dpad_up;
        dpadDownPressed = gamepad1.dpad_down;
        yPressed = gamepad1.y;
        aPressed = gamepad1.a;
        xPressed = gamepad1.x;

         //Setting LT to power of suck

            if (leftTrigger > .05 && leftBumper)
            {
                leftIntakePower = 0;
            }
            else if(leftTrigger > .05)
            {
                leftIntakePower = Math.pow(leftTrigger, 2)*.4;
            }
            else if(leftBumper)
            {
                leftIntakePower = -.7;
            }
            else
            {
                leftIntakePower = 0;
            }

            if (rightTrigger > .05 && rightBumper)
        {
            rightIntakePower = 0;
        }
        else if(rightTrigger > .05)
        {
            rightIntakePower = Math.pow(rightTrigger, 2)*.3;
        }
        else if(rightBumper)
        {
            rightIntakePower = -.7;
        }
        else
        {
            rightIntakePower = 0;
        }

        if(dpadDownPressed && dpadUpPressed)
        {
            panLiftingPower = 0;
        }
        else if(dpadUpPressed)
        {
            panLiftingPower = -1;
        }
        else if(dpadDownPressed)
        {
            panLiftingPower = .375;
        }
        else
        {
            panLiftingPower = 0;
        }

        if(yPressed && aPressed)
        {

        }
        else if(yPressed)
        {
            panSpinPosition = panSpinPosition + .03;
        }
        else if(aPressed)
        {
            panSpinPosition = panSpinPosition - .03;
        }
        else if(xPressed)
        {
            panSpinPosition = .25;
        }


            //Setting LT and RB to score or intake positions of pan


        telemetry.addData("end", currentStatus);
        telemetry.addData("startpress", startPressed);
        telemetry.addData("previous", previousStatus);



        //Setting dpad to panPosition
        //SET CONTROLS

        panSpinPosition = Range.clip(panSpinPosition,.1,.8);


        leftPanSpin.setPosition(panSpinPosition);
        rightPanSpin.setPosition(panSpinPosition);
        panLifterMotor.setPower(panLiftingPower);
        leftIntakeMotor.setPower(leftIntakePower);
        rightIntakeMotor.setPower(rightIntakePower);
        leftMotor.setPower(finLeftPower);
        rightMotor.setPower(finRightPower);
        //backLeftMotor.setPower(finBackPower);
        backRightMotor.setPower(finBackPower);

        telemetry.update();
    } // end loop
} // end class
