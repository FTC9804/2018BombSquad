//LIFTER created on September 8th by Isaac Dienstag

//Package statement
package org.firstinspires.ftc.teamcode.Teleop;

//Import statements
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

//Class declaration
public class Lifter extends TeleopMain { //This class has to extend TeleopMain in order to have access to getRuntime() method
    //Declare instance Motor
    private DcMotor liftMotor;

    //Declare booleans
    private boolean lifting = false, afterLift = false, doingItAll = false, droppingFromTop = false;
    private double time1, time2;
    private double variation = 0;
    private boolean PSIncrease = false, PSDecrease = false;

    //Declare digital channels
    private DigitalChannel atTop;
    private DigitalChannel atBottom;
    private Servo dumper;

    //Constructs a Lifter with 1 motor, 1 motor direction, and 3 Digital channels
    public Lifter(DcMotor lifter1, DcMotorSimple.Direction lifter1D, DigitalChannel topLimit, DigitalChannel bottomLimit, Servo dumpServo) {
        //Sets instance variables to inputs
        liftMotor = lifter1;
        liftMotor.setDirection(lifter1D);
        atTop = topLimit;
        atBottom = bottomLimit;
        dumper = dumpServo;
        dumper.scaleRange(0,1);
    } //Ends constructor

    //Asks for two doubles, and checks if input is too small to be considered intentional based on the value of tolereance.
    private double deadzone(double tolerance, double input){
        if(input > tolerance || input < -(tolerance)) //If |input| > deadzon
            return input; //We return input, because the input is likely intentional.
        else //Else (which means |input| <= |deadzone|)
            return 0; //e ignore input, and output 0.
    } //End double method

    //Returns true if |input| <= |tolerance|, false if |input| < |tolerance|
    private boolean outOfDeadzone(double tolerance, double input){
        return deadzone(tolerance, input) != 0;
    } //End boolean method

    //Asks for two doubles and two booleans, and uses them to determine the power of liftMotor
    public void lift(double lift, double drop, boolean doItAll, boolean dropFromTop){
        if(doItAll){doingItAll = true;}//Set a variable to true if doItAll is true for any amount of time
        if(dropFromTop){droppingFromTop = true;}//Set a variable to stay true if doItAll is true for any amount of time
        if(!atTop.getState() && !outOfDeadzone(.05, drop) && !doingItAll && !droppingFromTop){//If we are seeing the top Sensor and we are not dropping or raising
            time2 = this.getRuntime();
            lifting = true;//Set lifting to true, so it will automatically lift the rest of the way
        }

        if (outOfDeadzone(.05, lift) && outOfDeadzone(.05, drop)) { //If lift and drop are both > .05
            liftMotor.setPower(0); //Ignore their values and set the power of liftMotor to 0
            lifting = false; //Cancel all automatic movement
            doingItAll = false;
            droppingFromTop = false;
            afterLift = false;
        }
        else if (outOfDeadzone(.05, drop) && atBottom.getState()) { //If drop is a significant value and we are not at the bottom
            liftMotor.setPower(-drop / 2); //Set the power of liftMotor to half the value of drop
            afterLift = false; //Cancel all automatic movement
            lifting = false;
            doingItAll = false;
            droppingFromTop = false;
        }
        else if(lifting){ //If lifting is true, we want to lift the rest of the way automatically
            //(This is if we use lift to lift rather than do it all to lift)
            if(time1 - time2 < .5){ //If less than .5 seconds has passed since we last saw the atTop sensor
                time1 = this.getRuntime(); //Update the time1 time so the timer will update
                liftMotor.setPower(.8); //Set liftMotor power to .8
            }
            else { //Else (time1 - time2 >= .5)
                liftMotor.setPower(.1); //Set the liftMotor power to .1 so it holds at the top
                lifting = false; //Set lifting to false to move out of the if statement
                afterLift = true; //Set afterLift to true so we will hold power when we move out of this if statement
                doingItAll = false; //Set doingItAll to false so that it will not continue to try and doItAll
            }
        }
        else if (outOfDeadzone(.05, lift)) { //If lift is a significant value and we are not at the top
            liftMotor.setPower(lift);//Set the power of liftMotor to half the value of drop
            doingItAll = false; //Cancel all automatic movement
            droppingFromTop = false;
            afterLift = false;
        }
        else if(doingItAll && droppingFromTop){ //If we are trying to drop from top and do it all
            afterLift = false; //Cancel all automatic movement
            doingItAll = false;
            droppingFromTop = false;
        }
        else if(droppingFromTop){ //If we are trying to automatically drop from the time
            afterLift = false; //Set afterLift to false so it won't apply any power when we move out of this if statement
            if(atBottom.getState()) //If we are not at the bottom
                liftMotor.setPower(-.4); //Set liftMotor power to -.4
            else { //Else (!atBottom.getState())
                liftMotor.setPower(0); //Set the liftMotor power to 0
                droppingFromTop = false; //Set droppingFromTop to false so we move out of this else if statement
            }
        }
        else if(afterLift) //If afterLift is true and we are not trying to drop
            liftMotor.setPower(.1); //Set liftMotor power to .1, so we hold where we are and don't drop at all
        else if(doingItAll)//If we are trying to automatically raise from the bottom
            liftMotor.setPower(1); //Set the power to 1 to raise untill we hit the sensor
        else
            liftMotor.setPower(0); //Set the power of liftMotor to 0
    } //End void method

    public void lift2(double lift, double drop, boolean doItAll, boolean dropFromTop){
        if(doItAll){doingItAll = true;}//Set a variable to true if doItAll is true for any amount of time
        if(dropFromTop){droppingFromTop = true;}//Set a variable to stay true if doItAll is true for any amount of time

        if(!atTop.getState() && drop < .05 && !doingItAll && !droppingFromTop){//If we are seeing the top Sensor and we are not dropping or raising
            time1 = this.getRuntime();//Start running the two timer variables
            time2 = this.getRuntime();
            lifting = true;//Set lifting to true, so it will automatically lift the rest of the way
        }
        if(doingItAll && droppingFromTop){//If we are trying to drop and raise at the same time
            doingItAll = false;//Set both raise and drop to false
            droppingFromTop = false;
        }
        else if(droppingFromTop){ //If we are trying to automatically drop from the time
            if(outOfDeadzone(.05, lift)|| outOfDeadzone(.05, drop)){ //if drop or lift is > deadzone
                liftMotor.setPower(0); //Set the liftMotor power to 0
                afterLift = false; //Set afterLift to false so it won't apply any power when we move out of this if statement
                droppingFromTop = false; //Set droppingFromTop to false so we move out of this else if statement
            }
            else if(atBottom.getState()) //If we are not at the bottom
                liftMotor.setPower(-.4); //Set liftMotor power to -.4
            else { //Else (!atBottom.getState())
                liftMotor.setPower(0); //Set the liftMotor power to 0
                afterLift = false; //Set afterLift to false so it won't apply any power when we move out of this if statement
                droppingFromTop = false; //Set droppingFromTop to false so we move out of this else if statement
            }
        }
        else if(doingItAll){ //If we are trying to automatically raise from the bottom
            if(outOfDeadzone(.05,drop) || outOfDeadzone(.05,lift)){ //If drop or lift is > deadzone
                doingItAll = false; //Cancle doingItAll. This gives us an overide on doingItAll using the drop variable
                lifting = false;
            }
            if(!atTop.getState()){//If we are at the topSensor
                time1 = this.getRuntime();//Start running the two timer variables
                time2 = this.getRuntime();
                lifting = true;//Set lifting to true, so it will automatically lift the rest of the way
            }
            if(lifting) { //If lifting is true, we automatically want to raise the rest of the way
                if (time1 - time2 < .5) { //If less than .5 seconds has passed since we last saw the atTop sensor
                    time1 = this.getRuntime(); //Update the time1 time so the timer will update
                    liftMotor.setPower(.8); //Set liftMotor power to .8
                } else { //Else (time1 - time2 >= .5)
                    liftMotor.setPower(.1); //Set liftMotor power to .1 so it holds at the top
                    lifting = false; //Set lifting to false to move out of the if statement
                    afterLift = true; //Set afterLift to true so we will hold power when we move out of this if statement
                    doingItAll = false; //Set doingItAll to false so we move out of the larger if statement
                }
            }
            else { //Else (!lifting)
                liftMotor.setPower(1); //Set liftMotor power to 1
            }
        }
        else if(lifting){ //If lifting is true, we want to lift the rest of the way automatically
            //(This is if we use lift to lift rather than do it all to lift)
            if(time1 - time2 < .5){ //If less than .5 seconds has passed since we last saw the atTop sensor
                time1 = this.getRuntime(); //Update the time1 time so the timer will update
                liftMotor.setPower(.8); //Set liftMotor power to .8
            }
            else { //Else (time1 - time2 >= .5)
                liftMotor.setPower(.1); //Set the liftMotor power to .1 so it holds at the top
                lifting = false; //Set lifting to false to move out of the if statement
                afterLift = true; //Set afterLift to true so we will hold power when we move out of this if statement
            }
        }
        else if(afterLift && !outOfDeadzone(.05, drop)){ //If afterLift is true and we are not trying to drop
            liftMotor.setPower(.1); //Set liftMotor power to .1, so we hold where we are and don't drop at all
        }
        else if (outOfDeadzone(.05, lift) && outOfDeadzone(.05, drop)) //If lift and drop are both > .05
            liftMotor.setPower(0); //Ignore their values and set the power of liftMotor to 0
        else if (outOfDeadzone(.05, lift) && atTop.getState()) //If lift is a significant value and we are not at the top
            liftMotor.setPower(lift); //Set the power of liftMotor to the value lift
        else if (outOfDeadzone(.05, drop) && atBottom.getState()) { //If drop is a significant value and we are not at the bottom
            liftMotor.setPower(-drop / 2); //Set the power of liftMotor to half the value of drop
            afterLift = false; //Set afterLift to false so that dropping will cancel the afterLift
        }
        else //Else (lift && drop < .05 && !doItAll && !droppingFromTop)
            liftMotor.setPower(0); //Set the power of liftMotor to 0
    } //End void method

    public void lift3(double lift, double drop, boolean doItAll, boolean dropFromTop){
        if(doItAll){doingItAll = true;}//Set a variable to true if doItAll is true for any amount of time
        if(dropFromTop){droppingFromTop = true;}//Set a variable to stay true if doItAll is true for any amount of time
        if(doingItAll && droppingFromTop){//If we are trying to drop and raise at the same time
            doingItAll = false;//Set both raise and drop to false
            droppingFromTop = false;
            afterLift = false;
        }
        if(outOfDeadzone(.05, lift) || outOfDeadzone(.05, drop)){
            droppingFromTop = false;
            doingItAll = false;
        }
        if(doingItAll){
            if(!atTop.getState()){
                time1 = this.getRuntime();//Start running the two timer variables
                time2 = this.getRuntime();
                lifting = true;
            }
            if(lifting) { //If lifting is true, we automatically want to raise the rest of the way
                if (time1 - time2 < .5) { //If less than .5 seconds has passed since we last saw the atTop sensor
                    time1 = this.getRuntime(); //Update the time1 time so the timer will update
                    liftMotor.setPower(.8); //Set liftMotor power to .8
                } else { //Else (time1 - time2 >= .5)
                    liftMotor.setPower(.1); //Set liftMotor power to .1 so it holds at the top
                    lifting = false; //Set lifting to false to move out of the if statement
                    afterLift = true; //Set afterLift to true so we will hold power when we move out of this if statement
                    doingItAll = false; //Set doingItAll to false so we move out of the larger if statement
                }
            }
            else {
                liftMotor.setPower(.1);
            }
        }
        else if(droppingFromTop){ //If we are trying to automatically drop from the time
            if(outOfDeadzone(.05, lift)|| outOfDeadzone(.05, drop)){ //if drop or lift is > deadzone
                liftMotor.setPower(0); //Set the liftMotor power to 0
                afterLift = false; //Set afterLift to false so it won't apply any power when we move out of this if statement
                droppingFromTop = false; //Set droppingFromTop to false so we move out of this else if statement
            }
            else if(atBottom.getState()) //If we are not at the bottom
                liftMotor.setPower(-.4); //Set liftMotor power to -.4
            else { //Else (!atBottom.getState())
                liftMotor.setPower(0); //Set the liftMotor power to 0
                afterLift = false; //Set afterLift to false so it won't apply any power when we move out of this if statement
                droppingFromTop = false; //Set droppingFromTop to false so we move out of this else if statement
            }
        }
        else if(afterLift){ //If afterLift is true and we are not trying to drop
            liftMotor.setPower(.1); //Set liftMotor power to .1, so we hold where we are and don't drop at all
        }
        else if (outOfDeadzone(.05, lift) && outOfDeadzone(.05, drop)) //If lift and drop are both > .05
            liftMotor.setPower(0); //Ignore their values and set the power of liftMotor to 0
        else if (outOfDeadzone(.05, lift) && atTop.getState()) //If lift is a significant value and we are not at the top
            liftMotor.setPower(lift); //Set the power of liftMotor to the value lift
        else if (outOfDeadzone(.05, drop) && atBottom.getState()) //If drop is a significant value and we are not at the bottom
            liftMotor.setPower(-drop / 2); //Set the power of liftMotor to half the value of drop
        else //Else (lift && drop < .05 && !doItAll && !droppingFromTop)
            liftMotor.setPower(0); //Set the power of liftMotor to 0
    } //End void method

    //Asks for two booleans, and uses them to determine the power of the dumper servo
    public void dump(boolean dumps, boolean down, boolean increase, boolean decrease) {
        if (increase){ //if increase is true, signifying we would like to increase the pan servo value
            if (!PSIncrease) { //If previousStatus is false, meaning that we havn't changed the servo value on this press yet
                variation += .02; //Increment the servo value variable
                PSIncrease = true;//Set the mode to true so we will not run this increment again
            }
        }
        else //If increase is no longer true
            PSIncrease = false; //Set previousStatus to false so we will increment again on the next button press

        if(decrease){ //if decrease is true, signifying we would like to increase the pan servo value
            if(!PSDecrease) { //If previousStatus is false, meaning that we havn't changed the servo value on this press yet
                variation -= .02; //Decrieent the servo value variable
                PSDecrease = true; //Set the mode to true so we will not run this Decrement again
            }
        }
        else //If decrease is no longer true
            PSDecrease = false; //Set previousStatus to false so we will increment again on the next button press

        //meaning we would not like to change any mode boolean values
        if(down) //If down is true
            dumper.setPosition(.87+variation); //Set the position of dumper to .7
        else if(dumps) //Else if dumps is true
            dumper.setPosition(.65+variation); //Set the position of dumper to .4
        else //Else (!down && !dump)
            dumper.setPosition(.46+variation); //Set the position of dumper to .2
    } //End void method

    //Getters used for telemetry
    public double getLiftPower(){return liftMotor.getPower();} //Returns the current power of liftMotor
    public double getDumpPosition(){return dumper.getPosition();} //Returns the current position of the dumper servo
    public boolean getBottomState(){return atBottom.getState();} //Returns the state of the bottom sensor
    public boolean getTopState(){return atTop.getState();} //Returns the state of the top sensor
    public void setTime1(double time){time1 = time;}
} //Ends class