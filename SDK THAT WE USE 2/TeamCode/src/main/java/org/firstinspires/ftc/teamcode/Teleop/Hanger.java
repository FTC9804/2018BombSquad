//HANGER created by Isaac Dienstag on October 21st

//Package declaration
package org.firstinspires.ftc.teamcode.Teleop;

//Import statements
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//Class declaration
public class Hanger {

    //Private instance motor declaration
    private DcMotor hanger;

    //Private instance servo declaration
    private Servo swapper;

    //Constructs a Hanger and asks for one motor, one motor direction and a servo
    public Hanger(DcMotor hangingMotor, DcMotorSimple.Direction hangMotor, Servo swapper){
        hanger = hangingMotor;
        hanger.setDirection(hangMotor);
        this.swapper = swapper;
    }

    //Asks for two booleans and runs the hanger motor to different powers based whether drop and hang are true and false
    public void hangAndDrop(boolean  drop, boolean hang){
        if(drop && hang) //If drop and hang are both true
            hanger.setPower(0); //Stop running the hanger
        else if(drop) //Else if, drop is true (that means hang is not true)
            hanger.setPower(-.5); //Set the power of hanger to -.5
        else if(hang) //Else if, hang is true (that means drop is not true)
            hanger.setPower(.5); //Set the power of hanger to .5
        else //Else (that means hang and drop are both false)
            hanger.setPower(0); //Stop running the hanger
    } //End void method

    //Run the hanger motor to different powers based on the value of power
    public void hangAndDrop(double power){
        if(power > .05 || power < -.05) //If |power| > |.05|
            hanger.setPower(power); //Set the power of hanger to the value of power
        else //Else (-.05 <= power <= .05)
            hanger.setPower(0); //set the power of hanger to 0
    } //End void method

    //Asks for two booleans and changes the position of the swapper servo based on them
    public void swap(boolean up, boolean down){
        if(up) //If up is true
            swapper.setPosition(.05); //Set the position of swapper to .05
        else if(down) //If down is true
            swapper.setPosition(1); //Set the position of swapper to 1
    } //End void method

    //Getters used for telemetry
    public double getHangPower(){return hanger.getPower();} //Returns the power of the hanger motor
    public double getSwapperPosition(){return swapper.getPosition();} //Returns the position of the swapper
}


