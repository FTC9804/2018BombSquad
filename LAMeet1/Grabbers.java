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


@TeleOp(name = "Grabbers", group = "LAMeets")

public class Grabbers {

    //Servos we will use throughout class
    Servo spinServo, topServo, bottomServo, topSuckLeftServo, topSuckRightServo, bottomSuckLeftServo, bottomSuckRightServo;
    DcMotor upDownRight, upDownLeft;
    double topGrabberPosition = .25, bottomGrabberPosition = .25, spinServoPosition, topLeftPosition, topRightPosition, bottomLeftPosition, bottomRightPosition;
    boolean topStaySuck = false, topStaySpit = false, bottomStaySuck = false, bottomStaySpit = false;

    double liftPower = 0;

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
        bottomServo.setDirection(Servo.Direction.REVERSE);
        topSuckLeftServo.setDirection(Servo.Direction.FORWARD);
        topSuckRightServo.setDirection(Servo.Direction.REVERSE);
        bottomSuckLeftServo.setDirection(Servo.Direction.REVERSE);
        bottomSuckRightServo.setDirection(Servo.Direction.REVERSE);
        upDownLeft.setDirection(FORWARD);
        upDownRight.setDirection(REVERSE);
    }


    //Spins the grabbers 180
    public void spin(double spinValue) {

        if (spinValue<.5 && spinValue > .05)
        {
            spinValue -= spinValue/150;
        }
        else if (spinValue>=.5 && spinValue < 1)
        {
           spinValue += spinValue/150;
        }
        else
        {

        }

        spinServoPosition = spinValue;
        spinServoPosition = Range.clip(spinServoPosition, 0, 1);
    }
//
//    //sets the grabber to sideways position
//    public void spin90() {
//
//        spinServoPosition = 1/3;
//
//    }

    //in and out suck of grabbers
    public void topSuck() {

        topLeftPosition = .7;
        topRightPosition = .7;

    }

    public void topSpit() {
        topLeftPosition = 0.3;
        topRightPosition = 0.3;
    }

    public void topStill() {
        topLeftPosition = 0.5;
        topRightPosition = 0.5;
    }

    public void bottomStill() {
        bottomLeftPosition = 0.5;
        bottomRightPosition = 0.5;
    }

    public void bottomSuck() {
        bottomLeftPosition = .7;
        bottomRightPosition = .7;
    }
    public void bottomSpit() {
        bottomLeftPosition = 0.3;
        bottomRightPosition = 0.3;
    }
    public void topReOrientGlyph()
    {
        topLeftPosition = .7;
        topRightPosition = 0.3;
    }
    public void bottomReOrientGlyph()
    {
        bottomLeftPosition = .7;
        bottomRightPosition = 0.3;
    }


    //open and close of grabbers
    public void topOpen(double leftTriggerPower, boolean leftButton)
    {
        if(leftButton && leftTriggerPower >= .05) {
        }
        else if(leftTriggerPower >= .05) {
            topGrabberPosition += .006;
        }
        else if(leftButton)
        {
            topGrabberPosition -= .006;
        }
    }

    public void bottomOpen(double rightTriggerPower, boolean rightButton)
    {
        if(rightButton && rightTriggerPower >= .05) {
        }
        else if(rightTriggerPower >= .05) {
            bottomGrabberPosition += .006;
        }
        else if(rightButton)
        {
            bottomGrabberPosition -= .006;
        }
    }

    public void moveUp(double leftStickY)
    {
        liftPower = leftStickY;
        if (Math.abs(liftPower) < .05 )
        {
            liftPower = -0.1;
        }
        upDownLeft.setPower(liftPower);
        upDownRight.setPower(liftPower);
    }

    public void run()
    {
        spinServo.setPosition(spinServoPosition);
        topGrabberPosition = Range.clip(topGrabberPosition,0,.8);
        bottomGrabberPosition = Range.clip(bottomGrabberPosition,0,.83);
        topServo.setPosition(topGrabberPosition);
        bottomServo.setPosition(bottomGrabberPosition);
        topSuckLeftServo.setPosition(topLeftPosition);
        topSuckRightServo.setPosition(topRightPosition);
        bottomSuckLeftServo.setPosition(bottomLeftPosition);
        bottomSuckRightServo.setPosition(bottomRightPosition);
    }

}

