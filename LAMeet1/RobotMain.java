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

@TeleOp(name = "TeleOpCompetition", group = "LAMeets")


public class RobotMain extends OpMode {
    // Variables
    double rightPadY1;
    double leftPadX1;
    double rightPadX1;
    double leftStickY2;
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




    Drive drive;
    Grabbers grab;


    /* Initialize standard Hardware interfaces */
    public void init() { // use hardwaremap here instead of hwmap or ahwmap provided in sample code
        // Motor configurations in the hardware map
        DcMotor RightMotor = hardwareMap.dcMotor.get("rightMotor");
        DcMotor LeftMotor = hardwareMap.dcMotor.get("leftMotor");
        DcMotor FrontMotor = hardwareMap.dcMotor.get("topMotor");
        DcMotor BackMotor = hardwareMap.dcMotor.get("bottomMotor");
        drive = new Drive (RightMotor, LeftMotor, FrontMotor, BackMotor);

        DcMotor LeftLift = hardwareMap.dcMotor.get("liftMotorLeft");
        DcMotor RightLift = hardwareMap.dcMotor.get("liftMotorRight");
        Servo spin = hardwareMap.servo.get("spin");
        Servo top = hardwareMap.servo.get("openCloseTop");
        Servo bottom = hardwareMap.servo.get("openCloseBottom");
        Servo topSuckLeft = hardwareMap.servo.get("leftGrabberTop");
        Servo topSuckRight = hardwareMap.servo.get("rightGrabberTop");
        Servo bottomSuckLeft = hardwareMap.servo.get("leftGrabberBottom");
        Servo bottomSuckRight = hardwareMap.servo.get("leftGrabberBottom");
        grab = new Grabbers(LeftLift, RightLift, spin, top, bottom, topSuckLeft, topSuckRight, bottomSuckRight, bottomSuckLeft);

    }

    public void loop () {

        rightPadY1 = gamepad1.right_stick_y;
        rightPadX1 = gamepad1.right_stick_x;
        leftPadX1 = -(gamepad1.left_stick_x);

        leftStickY2 = gamepad2.left_stick_y;
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


        // If pads are moved
        boolean leftPadXOn = Math.abs (leftPadX1) > .05;
        boolean rightPadXOn = Math.abs (rightPadX1) > .05;
        boolean rightPadYOn = Math.abs (rightPadY1) > .05;

        // Combine rotation and movement
        if (leftPadXOn && !rightPadXOn && !rightPadYOn)
        {
            drive.turn(leftPadX1);
        }
        else
        {
            drive.move (rightPadX1, rightPadY1);
        }

        if(startButton2)
        {
            grab.spin();
            telemetry.addLine("Start Button Pressed");
        }

        if(backButton2)
        {
            grab.spin90();
            telemetry.addLine("Back Button Pressed");
        }


        //Top grabber suck controls
        if(leftDpadUp2)
        {
            grab.topSuck();
            telemetry.addLine("left D Pad up");
        }
        if(leftDpadDown2)
        {
            grab.topSpit();
            telemetry.addLine("left d pad down");
        }
        if(leftDpadRight2)
        {
            grab.topReOrientGlyph();
            telemetry.addLine("left d pad right");
        }

        //Bottom grabber suck controls
        if(rightDpadUp2)
        {
            grab.bottomSuck();
            telemetry.addLine("right d pad up");
        }
        if(rightDpadDown2)
        {
            grab.bottomSpit();
            telemetry.addLine("right d pad down");
        }
        if(rightDpadRight2)
        {
            grab.bottomReOrientGlyph();
            telemetry.addLine("right d pad down");
        }

        //opening and closing the grabbers
        grab.topOpen(leftTrigger2, leftButton2);
        grab.bottomOpen(rightTrigger2, rightButton2);

        //move grabbers up and down
        grab.moveUp(leftStickY2);

        drive.run();
        grab.run();
        telemetry.update();
    } // end loop
} // end class
