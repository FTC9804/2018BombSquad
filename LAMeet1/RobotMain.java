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


@TeleOp(name = "TeleOpCompetition", group = "LAMeets")


public class RobotMain extends OpMode {

    // Variables
    double rightPadY1;
    double leftPadX1;
    double rightPadX1;
    double leftStickY2;
    double rightStickX2;
    double leftTrigger2;
    double rightTrigger2;
    boolean startButton2;
    boolean backButton2;
    boolean rightButton2;
    boolean leftButton2;
    boolean leftDpadUp2;
    boolean rightDpadUp2;
    boolean rightDpadDown2;
    boolean leftDpadDown2;
    boolean leftDpadRight2;
    boolean rightDpadRight2;
    boolean rightPadXOn;
    boolean leftPadXOn;
    boolean rightPadYOn;

    boolean startPress;
    boolean xPress;

    double ltpress1;
    double rtpress1;
    boolean rbpress1;
    boolean lbpress1;
    boolean y1;
    boolean a1;

    boolean ltispressed1;
    boolean rtispressed1;



    // Motor configurations in the hardware map
    DcMotor RightMotor;
    DcMotor LeftMotor;
    DcMotor FrontMotor;
    DcMotor BackMotor;
    DcMotor LeftLift;
    DcMotor RightLift;
    Servo spin;
    Servo top;
    Servo bottom;
    Servo topSuckLeft;
    Servo topSuckRight;
    Servo bottomSuckLeft;
    Servo bottomSuckRight;
    Servo open;
    Servo rotate;
    DcMotor extend;
    Servo feeler;


    Drive drive;
    Grabbers grab;
    Relicc recovery;


    double spinValueAdjusted;


    /* Initialize standard Hardware interfaces */
    public void init() { // use hardwaremap here instead of hwmap or ahwmap provided in sample code
        // Motor configurations in the hardware map
        RightMotor = hardwareMap.dcMotor.get("rightMotor");
        LeftMotor = hardwareMap.dcMotor.get("leftMotor");
        FrontMotor = hardwareMap.dcMotor.get("topMotor");
        BackMotor = hardwareMap.dcMotor.get("bottomMotor");
        drive = new Drive (RightMotor, LeftMotor, FrontMotor, BackMotor);

        LeftLift = hardwareMap.dcMotor.get("liftMotorLeft");
        RightLift = hardwareMap.dcMotor.get("liftMotorRight");
        spin = hardwareMap.servo.get("spin");
        top = hardwareMap.servo.get("openCloseTop");
        bottom = hardwareMap.servo.get("openCloseBottom");
        topSuckLeft = hardwareMap.servo.get("leftGrabberTop");
        topSuckRight = hardwareMap.servo.get("rightGrabberTop");
        bottomSuckLeft = hardwareMap.servo.get("leftGrabberBottom");
        bottomSuckRight = hardwareMap.servo.get("rightGrabberBottom");
        grab = new Grabbers(LeftLift, RightLift, spin, top, bottom, topSuckLeft, topSuckRight, bottomSuckLeft, bottomSuckRight);

        open = hardwareMap.servo.get("grabRelic");
        rotate = hardwareMap.servo.get("liftRelic");
        extend = hardwareMap.dcMotor.get("grabberExtension");
        recovery = new Relicc(extend, open, rotate);

        feeler = hardwareMap.servo.get("feeler");


        spin.setPosition(0);
        top.setPosition(0.25);
        bottom.setPosition(0.25);
        topSuckLeft.setPosition(0.5);
        topSuckRight.setPosition(0.5);
        bottomSuckLeft.setPosition(0.5);
        bottomSuckRight.setPosition(0.5);
        feeler.setPosition(1);

    }

    public void loop () {

        rightPadY1 = gamepad1.right_stick_y;
        rightPadX1 = gamepad1.right_stick_x;
        leftPadX1 = -(gamepad1.left_stick_x);

        startPress = gamepad2.start;
        xPress = gamepad2.x;

        leftStickY2 = gamepad2.left_stick_y;
        rightStickX2 = gamepad2.right_stick_x;
        startButton2 = gamepad2.start;
        rightTrigger2 = gamepad2.right_trigger;
        leftTrigger2 = gamepad2.left_trigger;
        leftButton2 = gamepad2.left_bumper;
        rightButton2 = gamepad2.right_bumper;
        leftDpadUp2 = gamepad2.dpad_up;
        rightDpadUp2 = gamepad2.y;
        leftDpadDown2 = gamepad2.dpad_down;
        rightDpadDown2 = gamepad2.a;
        rightDpadRight2 = gamepad2.b;
        leftDpadRight2 = gamepad2.dpad_right;
        backButton2 = gamepad2.back;

        ltpress1 = gamepad1.left_trigger;
        rtpress1 = gamepad1.right_trigger;
        lbpress1 = gamepad1.left_bumper;
        rbpress1 = gamepad1.right_bumper;
        y1 = gamepad1.y;
        a1 = gamepad1.a;

        if(ltpress1 >= .05)
        {
            ltispressed1 = true;
        }
        else
        {
            ltispressed1 = false;
        }
        if (rtpress1 >= .05)
        {
            rtispressed1 = true;
        }
        else
        {
            rtispressed1 = false;
        }


        // If pads are moved
        if (Math.abs(leftPadX1) > 0.05)
        {
            leftPadXOn = true;
        }
        else
        {
            leftPadXOn = false;
        }
        if (Math.abs(rightPadX1) > 0.05)
        {
            rightPadXOn = true;
        }
        else
        {
            rightPadXOn = false;
        }
        if (Math.abs(rightPadY1) > 0.05)
        {
            rightPadYOn = true;
        }
        else
        {
            rightPadYOn = false;
        }

        // Combine rotation and movement
        if (leftPadXOn && !rightPadXOn && !rightPadYOn)
        {
            drive.turn (leftPadX1);
        }
        else
        {
            drive.move(rightPadX1, rightPadY1);
        }

//        if(startButton2)
//        {
//            grab.spin();
//            telemetry.addLine("Start Button Pressed");
//        }
//
//        if(backButton2)
//        {
//            grab.spin90();
//            telemetry.addLine("Back Button Pressed");
//        }


        grab.spin(startPress, xPress);

        //Top grabber suck controls
        if(leftDpadUp2)
        {
            grab.topSuck();
            telemetry.addLine("left D Pad up");
        }
        else if(leftDpadDown2)
        {
            grab.topSpit();
            telemetry.addLine("left d pad down");
        }
        else if(leftDpadRight2)
        {
            grab.topReOrientGlyph();
            telemetry.addLine("left d pad right");
        }
        else
        {
            grab.topStill();
        }


        //Bottom grabber suck controls
        if(rightDpadUp2)
        {
            grab.bottomSuck();
            telemetry.addLine("right d pad up");
        }
        else if(rightDpadDown2)
        {
            grab.bottomSpit();
            telemetry.addLine("right d pad down");
        }
        else if(rightDpadRight2)
        {
            grab.bottomReOrientGlyph();
            telemetry.addLine("right d pad down");
        }
        else
        {
            grab.bottomStill();
        }

        //opening and closing the grabbers
        grab.topOpen(leftTrigger2, leftButton2);
        grab.bottomOpen(rightTrigger2, rightButton2);

        //move grabbers up and down
        grab.moveUp(leftStickY2);

        if(ltispressed1 && !rtispressed1)
        {
            recovery.rotateUp();
        }
        else if(!ltispressed1 && rtispressed1 )
        {
            recovery.rotateDown();
        }

        if(rbpress1 && !lbpress1)
        {
            recovery.open();
        }
        else if(!rbpress1 && lbpress1)
        {
            recovery.close();
        }

        if(a1 && !y1)
        {
            recovery.extend();
        }
        else if(!a1 && y1)
        {
            recovery.retract();
        }




        drive.run();
        grab.run();
        recovery.run();
        telemetry.update();

    } // end loop
} // end class
