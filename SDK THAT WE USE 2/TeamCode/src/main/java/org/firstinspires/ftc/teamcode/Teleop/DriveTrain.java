//DRIVETRAIN created by Isaac Dienstag on September 8th

//Package statement
package org.firstinspires.ftc.teamcode.Teleop;

//Import Statements
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Auto.HelperClasses.FunctionsForAuto;

//Class declaration
public class DriveTrain {

    //Declare private instance motors
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    //Constructs a DriveTrain that asks for the names of the two motors, and the directions of the two motors
    public DriveTrain (DcMotor left, DcMotor right, DcMotorSimple.Direction leftDirection, DcMotorSimple.Direction rightDirection){
        //Set private instance DcMotors to the inputs given
        leftMotor = left;
        rightMotor = right;
        leftMotor.setDirection(leftDirection);
        rightMotor.setDirection(rightDirection);
    } //End constructor

    //Asks for two doubles, and checks if input is too small to be considered intentional based on the value of tolereance.
    private double deadzone(double tolerance, double input){
        if(input > tolerance || input < -(tolerance)) //If |input| > deadzon
            return input; //We return input, because the input is likely intentional.
        else //Else (which means |input| <= |deadzone|)
            return 0; //e ignore input, and output 0.
    } //End double method


    public void setZeroPow(){ //Sets the zero power behavior of both motors to BRAKE to make driving more precise
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    } //End void method

    //Sets the power of the left motor to the value of lsy
    public void setLeftPower(double lsy) {
        leftMotor.setPower(deadzone(.05, lsy));
    } //End void method

    //Sets the power of the right motor to the value of rsy
    public void setRightPower(double rsy) {
        rightMotor.setPower(deadzone(.05, rsy));
    } //End void method

    //Asks for two doubles and uses them to determine the power of the right and left motor
    public void driveFull(double leftPow, double rightPow){
        leftPow = leftPow*Math.abs(leftPow)*Math.abs(leftPow); //Set left power to the square of itself, maintaining sign
        rightPow = rightPow*Math.abs(rightPow)*Math.abs(rightPow); //Set right power to the square of itself, maintaining sign
        if(FunctionsForAuto.runFast){
            leftMotor.setPower(leftPow); //Set the power of leftMotor to leftPow
            rightMotor.setPower(rightPow); //Set the power of rightMotor to 3/4 of rightPow
        }
        else{
            leftMotor.setPower(leftPow*.75); //Set the power of leftMotor to 3/4 of leftPow
            rightMotor.setPower(rightPow*.75); //Set the power of rightMotor to 3/4 of rightPow
        }

    } //End void method

    //Getter statements used in telemetry
    public double getLeftPower(){ return leftMotor.getPower();} //Returns the current power of leftMotor
    public double getRightPower(){return rightMotor.getPower();} //Returns the current power of rightMotor
    public double getLeftPosition(){ return leftMotor.getCurrentPosition(); } //Returns the current position of leftMotor
    public double getRightPosition(){ return rightMotor.getCurrentPosition(); } //Returns the current position of rightMotor

} //End class