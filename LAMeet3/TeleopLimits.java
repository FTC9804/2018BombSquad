/**
 * TeleopV2 Made Saturday December 9 by Isaac, Marcus, and Steve
 */
//Import Statements

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.qualcomm.robotcore.util.Range.clip;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "TeleOpLimits", group = "LAMeets")
public class TeleopLimits extends OpMode {

    //Controls and Variables
    //Driving controls
    double leftStickX1; //Adjusted value taken from the raw value of the left X stick on gamepad 1
    double rightStickX1; //Adjusted value taken from the raw value of the right X stick on gamepad 1
    double rightStickY1; //Adjusted value taken from the raw value of the right Y stick on gamepad 1

    double timeOne;
    double timeTwo;

    // The IMU sensor object
    BNO055IMU imu;

    DistanceSensor sensorA;
    DistanceSensor sensorB;
    DistanceSensor sensorC;

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

    double outtakePower = -.5;

    boolean threeBlocks = false;

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

    boolean yPressed;
    boolean aPressed;
    boolean xPressed;
    boolean bPressed;
    // Motor and servo configurations
    DcMotor rightIntakeMotor; //Dc motor that controls the right intake/right wheel of the intake
    DcMotor leftIntakeMotor; //Dc motor that controls the left intake/left wheel of the intake
    DcMotor panLifterMotor;
    Servo leftPanSpin;
    Servo rightPanSpin;
    Servo panKick;
    Servo relicLongArm;
    Servo relicShortArm;
    Servo grab;
    Servo relicRotate;
    double grabPosition = 0;
    double relicLongArmPosition;
    double relicShortArmPosition;
    double relicRotatePosition = 0;

    DigitalChannel limitTop; //Touch sensor that tells us if we reach the top with the Pan
    DigitalChannel limitBottom; //Touch sensor that tells us if we reach the bottom with the Pan

    boolean endGame;
    boolean previousStatus;
    boolean currentStatus;


    /* Initialize standard Hardware interfaces */
    public void init() { // use hardwaremap here instead of hwmap or ahwmap provided in sample code


        BNO055IMU.Parameters IMUparameters = new BNO055IMU.Parameters();

        IMUparameters.mode                = BNO055IMU.SensorMode.IMU;
        IMUparameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        IMUparameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUparameters.calibrationDataFile = "BNO055IMUCalibration.json";
        IMUparameters.loggingEnabled      = true; //F A L S E???
        IMUparameters.loggingTag          = "IMU";
        IMUparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "i0");
        imu.initialize(IMUparameters);

        // Motor configurations in the hardware map
        rightMotor = hardwareMap.dcMotor.get("m1"); //RightMotor configuration
        leftMotor = hardwareMap.dcMotor.get("m2"); //LeftMotor configuration
        //backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor"); //BackLeftMotor configuration
        backRightMotor = hardwareMap.dcMotor.get("m3"); //BackRightMotor configuration
        rightIntakeMotor = hardwareMap.dcMotor.get("m4"); //rightIntakeMotor configuration
        leftIntakeMotor = hardwareMap.dcMotor.get("m5"); //leftIntakeMotor configuration
        panLifterMotor = hardwareMap.dcMotor.get("m6"); //panLifterMotor configuration

        // Servo configurations in the hardware map
        leftPanSpin = hardwareMap.servo.get("s1"); //leftPanSpin configuration
        rightPanSpin = hardwareMap.servo.get("s2"); //rightPanSpin configuration
        relicLongArm = hardwareMap.servo.get("s3");
        relicShortArm = hardwareMap.servo.get("s4");
        panKick = hardwareMap.servo.get("s5");
        grab = hardwareMap.servo.get("s6");
        relicRotate = hardwareMap.servo.get("s7");

        sensorA = hardwareMap.get(DistanceSensor.class, "i1");
        sensorB = hardwareMap.get(DistanceSensor.class, "i2");
        sensorC = hardwareMap.get(DistanceSensor.class, "i3");

        // Touch sensor configuration in the hardware map
        limitTop = hardwareMap.get(DigitalChannel.class, "d1"); //Top touchSensor configuration
        limitBottom = hardwareMap.get(DigitalChannel.class, "d2");

        // Motor directions
        rightMotor.setDirection(REVERSE); //Set rightMotor to FORWARD direction
        leftMotor.setDirection(FORWARD); //Set leftMotor to REVERSE direction
        backRightMotor.setDirection(REVERSE); //Set backRightMotor to REVERSE direcion
        //backLeftMotor.setDirection(REVERSE); //Set backLeftMotor to REVERSE direction
        rightIntakeMotor.setDirection(REVERSE); //Set rightIntakeMotor to FORWARD direction
        leftIntakeMotor.setDirection(FORWARD); //Set leftIntakeMotor to FORWARD direction
        panLifterMotor.setDirection(REVERSE); //Set panLifterMotor to FORWARD direction

        // Servo directions
        leftPanSpin.setDirection(Servo.Direction.REVERSE); //Set leftPanSpin to REVERSE direction
        rightPanSpin.setDirection(Servo.Direction.FORWARD); //Set rightPanSpin to FORWARD direction
        panKick.setDirection(Servo.Direction.FORWARD);
        grab.setDirection(Servo.Direction.REVERSE);
        relicRotate.setDirection(Servo.Direction.REVERSE);

        //Init powers
        rightMotor.setPower(0); //Set rightMotor to 0 power
        leftMotor.setPower(0); //Set leftMotor to 0 power
        backRightMotor.setPower(0); //Set backRightMotor to 0 power
        //backLeftMotor.setPower(0); //Set backLeftMotor to 0 power
        rightIntakeMotor.setPower(0); //Set rightIntakeMotor to 0 power
        leftIntakeMotor.setPower(0); //Set leftIntakeMotor to 0 power
        panLifterMotor.setPower(0); //Set panLifterMotor to 0 power

        grab.setPosition(0);
        relicRotate.setPosition(0);

        leftPanSpin.setPosition(.1875);
        rightPanSpin.setPosition(.1875);




        endGame = false;
        previousStatus = false;
        currentStatus = false;
    }
    public void loop () {

        //DRIVING

        telemetry.addData("Left X Joy Raw: ", gamepad1.left_stick_x); //The raw value of left stick x
        telemetry.addData("Right X Joy Raw: ", gamepad1.right_stick_x); //The raw value of right stick x
        telemetry.addData("Right Y Joy Raw: ", gamepad1.right_stick_y); //The raw value of right stick y

        leftStickX1 = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);
        //Set rightStickX1 to the left stick x value of gamepad 1 times the absolute value of this left stick x value
        rightStickX1 = gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x);
        //Set rightStickX1 to the right stick x value of gamepad 1 times the absolute value of this right stick x value
        rightStickY1 = gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y);
        //Set rightStickY1 to the right stick x value of gamepad 1 times the absolute value of this right stick y value

        yPressed = gamepad1.y;
        aPressed = gamepad1.a;
        xPressed = gamepad1.x;
        bPressed = gamepad1.b;

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
                    finLeftPower = testLeftPower / maxPower;
                    finRightPower = testRightPower / maxPower;
                    finFrontPower = testFrontPower / maxPower;
                    finBackPower = testBackPower / maxPower;
                }
                break;
            default:
                telemetry.addData("Something", "Messed up");
                break;
        }

        if (Math.abs(gamepad1.right_stick_x) > 0.05)
        {
            finLeftPower = finLeftPower+ .3*gamepad1.right_stick_x;
            finRightPower = finRightPower - .3*gamepad1.right_stick_x;

        }

        if (Math.abs(gamepad1.left_stick_x) > 0.05)
        {
            finLeftPower /= 1.75;
            finRightPower /= 1.75;
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

        if(!currentStatus)
        {
            if (sensorA.getDistance(DistanceUnit.CM) < 14 && sensorB.getDistance(DistanceUnit.CM) < 14 && sensorC.getDistance(DistanceUnit.CM) < 14)
            {
                leftIntakePower = -.5;
                rightIntakePower = -.5;
            }
            else {
                //Setting LT to power of suck
                if (leftTrigger > .05 && leftBumper) {
                    leftIntakePower = 0;
                    rightIntakePower = 0;
                } else if (leftTrigger > .05) {
                    leftIntakePower = Math.pow(leftTrigger, 2) * .69;
                    rightIntakePower = Math.pow(leftTrigger, 2);
                } else if (leftBumper) {
                    leftIntakePower = -.7;
                    rightIntakePower = -.7;
                } else {
                    leftIntakePower = 0;
                    rightIntakePower = 0;
                }
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
                panLiftingPower = 1;
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
                panSpinPosition = panSpinPosition + .05;
            }
            else if(aPressed)
            {
                panSpinPosition = panSpinPosition - .05;
            }
            else if(xPressed)
            {
                panSpinPosition = .3;
            }
            else if(bPressed)
            {
                panSpinPosition = .1875;
            }

            if (dpadLeftPressed)
            {
                score();
            }

            telemetry.addData("panLifting", panLiftingPower);
            telemetry.addData("limitTop", limitTop.getState());
            telemetry.addData("limitBottom", limitBottom.getState());

            panSpinPosition = Range.clip(panSpinPosition,.1875,.6);


        }
        //When in end game mode
        else
        {
//            //While dpad right is pressed
//            if(dpadRightPressed)
//            {
//                //If relicLongArmAngle is in the right spot, do nothing
//                if (longArmAngle <= 185 && longArmAngle >= 175)
//                {
//                    relicLongArm.setPosition(0);
//                }
//                //If the long arm is below, move up
//                else if(longArmAngle < 175)
//                {
//                    relicLongArm.setPosition(.5);
//                }
//                //if the long arm is above, move down
//                else if(longArmAngle > 185)
//                {
//                    relicLongArm.setPosition(-.5);
//                }
//                //just being safe, this should never happen
//                else
//                {
//                    relicLongArm.setPosition(0);
//                }
//                //If short arm is in the right place, do nothing
//                if(shortArmAngle <= 275 && shortArmAngle >= 265)
//                {
//                    relicShortArm.setPosition(0);
//                }
//                //If short arm is above, move down
//                else if(shortArmAngle > 275)
//                {
//                    relicShortArm.setPosition(-.5);
//                }
//                //If short arm is below, move up
//                else if(shortArmAngle < 265)
//                {
//                    relicShortArm.setPosition(.5);
//                }
//                //Being safe
//                else
//                {
//                    relicShortArm.setPosition(0);
//                }
//            }
//            else if(dpadRightPressed)
//            {
//                //If relicLongArmAngle is in the right spot, do nothing
//                if(longArmAngle <= 95 && longArmAngle >= 85)
//                {
//                    relicLongArm.setPosition(0);
//                }
//                //If the long arm is below, move up
//                else if(longArmAngle < 85)
//                {
//                    relicLongArm.setPosition(.5);
//                }
//                //if the long arm is above, move down
//                else if(longArmAngle > 95)
//                {
//                    relicLongArm.setPosition(-.5);
//                }
//                //just being safe, this should never happen
//                else
//                {
//                    relicLongArm.setPosition(0);
//                }
//                //If short arm is in the right place, do nothing
//                if(shortArmAngle <= 275 && shortArmAngle >= 265)
//                {
//                    relicShortArm.setPosition(0);
//                }
//                //If short arm is above, move down
//                else if(shortArmAngle > 275)
//                {
//                    relicShortArm.setPosition(-.5);
//                }
//                //If short arm is below, move up
//                else if(shortArmAngle < 265)
//                {
//                    relicShortArm.setPosition(.5);
//                }
//                //Being safe
//                else
//                {
//                    relicShortArm.setPosition(0);
//                }
//            }
//            else if(dpadDownPressed)
//            {
//                //If relicLongArmAngle is in the right spot, do nothing
//                if(longArmAngle <= 245 && longArmAngle >= 235)
//                {
//                    relicLongArm.setPosition(0);
//                }
//                //If the long arm is below, move up
//                else if(longArmAngle < 235)
//                {
//                    relicLongArm.setPosition(.5);
//                }
//                //if the long arm is above, move down
//                else if(longArmAngle > 245)
//                {
//                    relicLongArm.setPosition(-.5);
//                }
//                //just being safe, this should never happen
//                else
//                {
//                    relicLongArm.setPosition(0);
//                }
//                //If short arm is in the right place, do nothing
//                if(shortArmAngle <= 125 && shortArmAngle >= 115)
//                {
//                    relicShortArm.setPosition(0);
//                }
//                //If short arm is above, move down
//                else if(shortArmAngle > 125)
//                {
//                    relicShortArm.setPosition(-.5);
//                }
//                //If short arm is below, move up
//                else if(shortArmAngle < 115)
//                {
//                    relicShortArm.setPosition(.5);
//                }
//                //Being safe
//                else
//                {
//                    relicShortArm.setPosition(0);
//                }
//            }

            if (gamepad1.left_trigger > 0.3) {
                relicLongArmPosition = (gamepad1.left_trigger + .5) * (2/3);
                relicLongArm.setPosition( relicLongArmPosition );
            }
            else if (gamepad1.left_bumper) {
                relicLongArm.setPosition( 0 );
            }
            else
            {
                relicLongArm.setPosition( .5 );
            }

            if (gamepad1.right_trigger > 0.3) {
                relicShortArmPosition = (gamepad1.right_trigger + .5) * (2/3);
                relicShortArm.setPosition( relicShortArmPosition );
            }
            else if (gamepad1.right_bumper) {
                relicShortArm.setPosition( 0 );
            }
            else
            {
                relicShortArm.setPosition( .5 );
            }

            if (gamepad1.a)
            {
                grabPosition += .001;
                grabPosition = Range.clip(grabPosition, 0, 1);
                grab.setPosition( grabPosition );
            }
            else if (gamepad1.y)
            {
                grabPosition -= .001;
                grabPosition = Range.clip(grabPosition, 0, 1);
                grab.setPosition( grabPosition );
            }

            if (gamepad1.x)
            {
                relicRotatePosition += .001;
                relicRotatePosition = Range.clip(relicRotatePosition, 0, 1);
                relicRotate.setPosition( relicRotatePosition );
            }
            else if (gamepad1.b)
            {
                relicRotatePosition -= .001;
                relicRotatePosition = Range.clip(relicRotatePosition, 0, 1);
                relicRotate.setPosition( relicRotatePosition );
            }

        }

        telemetry.addData("end", currentStatus);
        telemetry.addData("startpress", startPressed);
        telemetry.addData("previous", previousStatus);


        // finLeftPower = Math.pow(finLeftPower,2);
        //finRightPower = Math.pow(finRightPower,2);
        //finBackPower = Math.pow(finBackPower,2);

        //Setting dpad to panPosition
        //SET CONTROLS
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

    public void score ()
    {
        while (limitTop.getState())
        {
            panLifterMotor.setPower(-.8);
        }
        panLifterMotor.setPower(0);
        panLifterMotor.setPower(0);
        leftPanSpin.setPosition(.6);
        rightPanSpin.setPosition(.6);
        pause(.8);
        leftPanSpin.setPosition(.15);
        rightPanSpin.setPosition(.15);
        while (limitBottom.getState())
        {
            panLifterMotor.setPower(.4);
        }
        panLifterMotor.setPower(0);
        leftPanSpin.setPosition(.05);
        rightPanSpin.setPosition(.05);
    }

    public void pause( double time ) {
        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < time) {
            timeTwo = this.getRuntime();
        }
    }



} // end class
