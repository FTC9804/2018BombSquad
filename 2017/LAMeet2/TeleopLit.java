    package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;


@TeleOp(name = "TeleOpCompetitionLit", group = "LAMeets")


public class RobotRegular extends OpMode {

    //Controls and Variables

    //Driving controls
    double leftStickX1;
    double leftStickY1;
    double rightStickX1;

    //Driving variables
    boolean single = true;
    int leftOverOne;
    int rightOverOne;
    int frontOverOne;
    int backOverOne;
    double rotRatio = .733333;
    int direction = 0;
    int over;

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

    double finLeftPower = 0;
    double finRightPower = 0;
    double finFrontPower = 0;
    double finBackPower = 0;

    //Grabber variables
    double topGrabberPosition = .25, topLeftPosition, topRightPosition;

    double liftPower = 0;

    //Grabber controls
    double rightTrigger1;
    boolean rightBumper1;
    boolean y1;
    boolean a1;
    boolean x1;
    boolean b1;
    boolean start;

    //Relicc variables
    double openPosition, rotatePosition, extendPower;

    //Relicc controls
    boolean dpadUp1;
    boolean dpadDown1;
    boolean dpadRight1;
    boolean dpadLeft1;

    double leftTrigger1;
    boolean leftBumper1;

    // Motor configurations in the hardware map
    DcMotor RightMotor;
    DcMotor LeftMotor;
    DcMotor FrontMotor;
    DcMotor BackMotor;
    DcMotor LeftLift;
    DcMotor RightLift;
    Servo top;
    Servo topSuckLeft;
    Servo topSuckRight;
    Servo open;
    Servo rotate;
    DcMotor extension;

    Servo feelerRaise;
    Servo feelerSwipe;



    /* Initialize standard Hardware interfaces */
    public void init() { // use hardwaremap here instead of hwmap or ahwmap provided in sample code
        // Motor configurations in the hardware map
        RightMotor = hardwareMap.dcMotor.get("rightMotor");
        LeftMotor = hardwareMap.dcMotor.get("leftMotor");
        FrontMotor = hardwareMap.dcMotor.get("topMotor");
        BackMotor = hardwareMap.dcMotor.get("bottomMotor");

        LeftLift = hardwareMap.dcMotor.get("liftMotorLeft");
        RightLift = hardwareMap.dcMotor.get("liftMotorRight");
        top = hardwareMap.servo.get("openCloseTop");
        topSuckLeft = hardwareMap.servo.get("leftGrabberTop");
        topSuckRight = hardwareMap.servo.get("rightGrabberTop");


        open = hardwareMap.servo.get("grabRelic");
        rotate = hardwareMap.servo.get("liftRelic");
        extension = hardwareMap.dcMotor.get("grabberExtension");

        feelerRaise = hardwareMap.servo.get("feeler raise");
        feelerSwipe = hardwareMap.servo.get("feeler swipe");

        // Motor directions: set forward/reverse
        RightMotor.setDirection(REVERSE);
        LeftMotor.setDirection(FORWARD);
        FrontMotor.setDirection(FORWARD);
        BackMotor.setDirection(REVERSE);

        top.setDirection(Servo.Direction.FORWARD);
        topSuckLeft.setDirection(Servo.Direction.FORWARD);
        topSuckRight.setDirection(Servo.Direction.REVERSE);
        LeftLift.setDirection(REVERSE);
        RightLift.setDirection(FORWARD);

        top.setPosition(.5);
        topSuckLeft.setPosition(1);
        topSuckRight.setPosition(1);

        feelerRaise.setPosition(1);
        feelerSwipe.setPosition(.5);

    }

    public void loop () {


        //DRIVING
        //DRIVING
        //DRIVING
        //DRIVING
        //DRIVING

        leftStickX1 = gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x);
        leftStickY1 = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x); //i know this is reversed
        rightStickX1 = gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y);

        telemetry.addData("Left X Joy Raw: ", gamepad1.left_stick_x);
        telemetry.addData("Left Y Joy Raw: ", gamepad1.left_stick_y);
        telemetry.addData("Right X Joy Raw: ", gamepad1.right_stick_x);

        linLeftPower = leftStickY1;
        linRightPower = -leftStickY1;
        linFrontPower = -leftStickX1;
        linBackPower = leftStickX1;

        rotLeftPower = -rightStickX1;
        rotRightPower = -rightStickX1;
       // rotBackPower = -rightStickX1 * rotRatio;
        //rotFrontPower = -rightStickX1 * rotRatio;

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


        //GRABBING


        y1 = gamepad1.y;
        a1 = gamepad1.a;

        leftTrigger1 = gamepad1.left_trigger;
        leftBumper1 = gamepad1.left_bumper;

        rightTrigger1 = gamepad1.right_trigger;
        rightBumper1= gamepad1.right_bumper;
        start = gamepad1.start;

        //Top grabber suck controls
        if(leftBumper1)
        {
            topLeftPosition = .7;
            topRightPosition = .7;
            telemetry.addLine("Left Bumper");
        }
        else if(leftTrigger1>.5)
        {
            topLeftPosition = 0.3;
            topRightPosition = 0.3;
            telemetry.addLine("Left Trigger");
        }
        else if(start)
        {
            topLeftPosition = .7;
            topRightPosition = 0.3;
            telemetry.addLine("Start");
        }
        else
        {
            topLeftPosition = 0.5;
            topRightPosition = 0.5;
        }

        //opening and closing the grabbers
        if(rightBumper1 && rightTrigger1 >= .05) {
        }
        else if(rightTrigger1 >= .05) {
            topGrabberPosition += .006;
        }
        else if(rightBumper1)
        {
            topGrabberPosition -= .006;
        }
        //move grabbers up and down
        if (y1)
        {
            liftPower = .6;
        }
        else if (a1)
        {
            liftPower = -.6;
        }
        else
        {
            liftPower = 0;
        }
        LeftLift.setPower(liftPower);
        RightLift.setPower(liftPower);


        //RELICC
        //RELICC
        //RELICC
        //RELICC
        //RELICC
        //RELICC

        x1 = gamepad1.x;
        b1 = gamepad1.b;

        dpadLeft1=gamepad1.dpad_left;
        dpadRight1=gamepad1.dpad_right;
        dpadUp1=gamepad1.dpad_up;
        dpadDown1=gamepad1.dpad_down;

        if(b1 && !x1)
        {
            rotatePosition = 0;
        }
        else if(!b1 && x1)
        {
            rotatePosition = 1;
        }

        if(dpadRight1 && !dpadLeft1)
        {
            openPosition = 0;
        }
        else if(!dpadRight1 && dpadLeft1)
        {
            openPosition = 1;
        }

        if(dpadDown1 && !dpadUp1)
        {
            extendPower = -1;
        }
        else if(!dpadDown1 && dpadUp1)
        {
            extendPower = 1;
        }
        else
        {
            extendPower = 0;
        }




        //SET CONTROLS
        //SET CONTROLS
        //SET CONTROLS
        //SET CONTROLS
        //SET CONTROLS

        finBackPower = Range.clip(finBackPower, -1, 1);
        finFrontPower = Range.clip(finFrontPower, -1, 1);
        finRightPower = Range.clip(finRightPower, -1, 1);
        finLeftPower = Range.clip(finLeftPower, -1, 1);

        if ((finBackPower<0&&finRightPower>0)||(finBackPower>0&&finRightPower<0))
        {
            if (Math.abs(finFrontPower)>.1)
            {
                finRightPower/=4;
                finLeftPower/=4;
            }
        }

        LeftMotor.setPower(-finLeftPower);
        RightMotor.setPower(-finRightPower);
        FrontMotor.setPower(finFrontPower);
        BackMotor.setPower(finBackPower);

        topGrabberPosition = Range.clip(topGrabberPosition,0,.25);
        top.setPosition(topGrabberPosition);
        topSuckLeft.setPosition(topLeftPosition);
        topSuckRight.setPosition(topRightPosition);

        open.setPosition(openPosition);
        rotate.setPosition(rotatePosition);
        extension.setPower(extendPower);

        telemetry.addData("topgrabberposition", topGrabberPosition);
        telemetry.update();

    } // end loop
} // end class
