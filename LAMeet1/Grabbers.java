package org.firstinspires.ftc.teamcode;

/**
 * Created by Isaac Dienstag, and Mathew Redford on 11/18/17.
 */

import com  .qualcomm.robotcore.eventloop.opmode.OpMode;
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


@TeleOp(name = "OmniDrive", group = "OmniDriveBot")

public class Grabbers {

    //Servos we will use throughout class
    Servo spinServo, topServo, bottomServo, topSuckLeftServo, topSuckRightServo, bottomSuckLeftServo, bottomSuckRightServo;
    DcMotor upDownRight, upDownLeft;
    double topGrabberPosition = .5, bottomGrabberPosition = .5, spinServoPosition, topLeftPosition, topRightPosition, bottomLeftPosition, bottomRightPosition;
    boolean topStaySuck = false, topStaySpit = false, bottomStaySuck = false, bottomStaySpit = false;




    //Making an object of the class Grabbers
    public Grabbers (DcMotor newUpDownLeft, DcMotor newUpDownRight, Servo newSpin, Servo newTop, Servo newBottom, Servo newTopSuckLeft, Servo newTopSuckRight, Servo newBottomSuckLeft, Servo newBottomSuckRight) {
        //setting Servos to input servos
        spinServo = newSpin;
        topServo = newTop;
        bottomServo = newBottom;
        topSuckLeftServo = newTopSuckLeft;
        topSuckRightServo = newTopSuckRight;
        bottomSuckLeftServo = newBottomSuckLeft;
        bottomSuckRightServo = newBottomSuckRight;

        upDownLeft = newUpDownLeft;
        upDownRight = newUpDownRight;

        //Setting servo directions
        spinServo.setDirection(Servo.Direction.FORWARD);
        topServo.setDirection(Servo.Direction.FORWARD);
        bottomServo.setDirection(Servo.Direction.FORWARD);
        topSuckLeftServo.setDirection(Servo.Direction.FORWARD);
        topSuckRightServo.setDirection(Servo.Direction.FORWARD);
        bottomSuckLeftServo.setDirection(Servo.Direction.FORWARD);
        bottomSuckRightServo.setDirection(Servo.Direction.FORWARD);
        upDownLeft.setDirection(FORWARD);
        upDownRight.setDirection(FORWARD);
    }


    //Spins the grabbers 180
    public void spin() {


        if(spinServo.getPosition() <= .10) {
            spinServoPosition = 2/3;
        }
        else {
            spinServoPosition = 0;
        }

    }

    //sets the grabber to sideways position
    public void spin90() {

        spinServoPosition = 1/3;


    }



    //in and out suck of grabbers
    public void topSuck() {

        topLeftPosition = 1;
        topRightPosition = 1;

    }
    public void topSpit() {
        topLeftPosition = 0;
        topRightPosition = 0;
    }

    public void bottomSuck() {
        bottomLeftPosition = 1;
        bottomRightPosition = 1;
    }
    public void bottomSpit() {
        bottomLeftPosition = 0;
        bottomRightPosition = 0;
    }
    public void topReOrientGlyph()
    {
        topLeftPosition = 1;
        topRightPosition = 0;
    }
    public void bottomReOrientGlyph()
    {
        bottomLeftPosition = 1;
        bottomRightPosition = 0;
    }



    //open and close of grabbers
    public void topOpen(double leftTriggerPower, boolean leftButton)
    {
        if(leftButton && leftTriggerPower >= .05) {
        }
        else if(leftTriggerPower >= .05) {
            topGrabberPosition += .02;
        }
        else if(leftButton)
        {
            topGrabberPosition -= .02;
        }
    }

    public void bottomOpen(double rightTriggerPower, boolean rightButton)
    {
        if(rightButton && rightTriggerPower >= .05) {
        }
        else if(rightTriggerPower >= .05) {
            topGrabberPosition += .02;
        }
        else if(rightButton)
        {
            bottomGrabberPosition -= .02;
        }
    }



    public void moveUp(double leftStickY)
    {
        upDownLeft.setPower(leftStickY);
        upDownRight.setPower(leftStickY);
    }



    public void run()
    {
        spinServo.setPosition(spinServoPosition);
        topServo.setPosition(topGrabberPosition);
        bottomServo.setPosition(bottomGrabberPosition);
        topSuckLeftServo.setPosition(topLeftPosition);
        topSuckRightServo.setPosition(topRightPosition);
        bottomSuckLeftServo.setPosition(bottomLeftPosition);
        bottomSuckRightServo.setPosition(bottomRightPosition);
    }



}


