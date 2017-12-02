
//package declaration
package org.firstinspires.ftc.teamcode;

//import statements
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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



public class Relicc {


    DcMotor extension;
    Servo open, rotate;

    double openPosition, rotatePosition, extendPower;

    public Relicc(DcMotor newExtension, Servo newOpen, Servo newRotate)
    {

    }

    public void close()
    {
        openPosition = 0;
    }
    public void open()
    {
        openPosition = 1;
    }
    public void rotateUp()
    {
        rotatePosition = 0;
    }
    public void rotateDown()
    {
        rotatePosition = 1;
    }
    public void extend()
    {
        extendPower = 1;
    }
    public void retract()
    {
        extendPower = -1;
    }

    public void run()
    {
        open.setPosition(openPosition);
        rotate.setPosition(rotatePosition);
        extension.setPower(extendPower);
    }
}
