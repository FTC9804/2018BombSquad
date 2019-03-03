//TELEOPMAIN created on September 8th by Isaac Dienstag

//Package statements
package org.firstinspires.ftc.teamcode.Teleop;

//Import statements
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

//Declaration for display on the driver station
@TeleOp(name = "TeleopMain")

public class TeleopMain extends OpMode {


    DriveTrain DT; Grabber GB; Lifter LT; Hanger HG;//Robot object declarations
    int loopCounter; double timeOne, timeTwo;//Telemetry variables

    //Init method required by OpMode. Initializes all of our RobotObjects and initializes the servo positions where needed
    public void init(){
        //Call the constructors for our Objects with the correct hardwareMap positions and directions
        DT = new DriveTrain(hardwareMap.dcMotor.get("m2"), hardwareMap.dcMotor.get("m1"), REVERSE, FORWARD);
        GB = new Grabber(hardwareMap.dcMotor.get("m3"), hardwareMap.dcMotor.get("m4"), FORWARD, REVERSE,
                hardwareMap.digitalChannel.get("d1"), hardwareMap.digitalChannel.get("d5"), hardwareMap.crservo.get("s3"));
        LT = new Lifter(hardwareMap.dcMotor.get("m5"), REVERSE, hardwareMap.get(DigitalChannel.class, "d3"),
                hardwareMap.get(DigitalChannel.class, "d2"), hardwareMap.servo.get("s1"));
        HG = new Hanger(hardwareMap.dcMotor.get("m6"), FORWARD, hardwareMap.servo.get("s2"));

        //Initialize servo positions and motor zero power behaviors
        LT.dump(false, true, false, false);//Set the dumper servo to stay down
        DT.setZeroPow();//Set the zero power behavior of the DriveTrain motors to BRAKE so we have more precise control
        GB.setZeroPow();//Set the zero power behavior of the sweeper to BRAKE so we have more precise control
    }

    //Set timeOne to this.getRuntime() only when we press start
    public void start() {timeOne = this.getRuntime(); LT.setTime1(this.getRuntime()); loopCounter = 0;}

    //Loop method required by OpMode. This is what runs for the duration of the teleop period
    public void loop(){
        //Increment the loopCounter and update timeTwo continuously
        loopCounter++;
        timeTwo = this.getRuntime();

        //Set DriveTrain powers to left stick and right stick y of gamepad1
        DT.driveFull(gamepad1.left_stick_y,gamepad1.right_stick_y);

        //Set reach control to left and right triggers of gamepad 1
        //Also set the disrupter toggle control to the b button of gamepad2
        GB.reach(gamepad1.left_trigger, gamepad1.right_trigger, gamepad2.b);
        //Set intake controls to the Y axis of the left stick on gamepad2
        GB.intake(gamepad2.left_stick_y);
        //Set the direct control of disrupter control to the a button on gamepad2
        GB.disrupt(gamepad2.a);

        //At the very beginning of teleop (in the first half second), we want to run the lifter up and move the dumper servo
        //to the neutral position in order to get the dumper into the neutral position from the init position
        if(timeTwo - timeOne < .5)
            //If we are in the first .5 seconds of teleop
            LT.lift(.5,0,false,false); //Set the lift power of the lifter to .5 to move the lifter up
        else if(timeTwo - timeOne < 1)//Else if we are in the first second of teleop
            //Set the power of the lifter to -.3 to move the lifter back down to the bottom
            LT.lift(0,.3,false, false);
        else //Else (if we are not in the first second of teleop)
            //Set the lifting power to the triggers of gamepad2
            LT.lift2(gamepad2.left_trigger, gamepad2.right_trigger,gamepad2.left_bumper,gamepad2.right_bumper);

        if(timeTwo - timeOne < .3) //If we are in the first .3 seconds of teleop
            LT.dump(false, true, false, false); //Hold the dumper in the down position
        else //Else
            //Set the control of the dumper servo to the x and y buttons of gamepad2
            LT.dump(gamepad2.x, gamepad2.y, gamepad2.dpad_up, gamepad2.dpad_down);

        //Set hanging motor power control to the y axis of the gunner's right stick
        HG.hangAndDrop(gamepad2.right_stick_y);
        //Set the up and down positions of the hanging lock mechanism to up and down on the gunner's dpad respectively
        HG.swap(gamepad2.dpad_left, gamepad2.dpad_right);

        //Telemetry for testing
        telemetry.addLine("Timers: "); //Timer header
        telemetry.addData("LPS", loopCounter/(timeTwo-timeOne)); //Loops per second value
        telemetry.addData("Time elapsed: ", Math.round((timeTwo - timeOne)*100)/100.0); //The time elapsed in our teleop
        telemetry.addData("Left Power: ", DT.getLeftPower()); //The power of the left motor
        telemetry.addLine(); //Gap between time telemetry and motor telemetry
        telemetry.addLine("Motors: "); //Motor header
        telemetry.addData("Right Power: ", DT.getRightPower()); //The power of the right motor
        telemetry.addData("Extender Power: ", GB.getExtenderPower()); //The power of the extender
        telemetry.addData("Sweeper Power: ", GB.getSweeperPower()); //The power of the sweeper
        telemetry.addData("Lift Power: ", LT.getLiftPower()); //The power of the lifter
        telemetry.addData("Hanger Power: ", HG.getHangPower()); //The power of the hanging motor
        telemetry.addLine(); //Gap between time Motor and servo telemetry
        telemetry.addLine("Servos: "); //Servo header
        telemetry.addData("Disrupter Power: ", GB.getDisrupterPower()); //The power of the disrupter CR servo
        telemetry.addData("Dump Position:", LT.getDumpPosition()); //The poisition of the dump servo
        telemetry.addData("Swapper Position:", HG.getSwapperPosition()); //The position of the swapper servo
        telemetry.addLine(); //Gap between servo telemetry and sensor telemetry
        telemetry.addLine("Sensors: "); //Sensor header
        telemetry.addData("Slow State: ", GB.getSlowState()); //The current state of the slow sensor
        telemetry.addData("Top State: ", LT.getTopState()); //The current state of the  atTop sensor
        telemetry.addData("Bottom State: ", LT.getBottomState()); //The current state of the atBottom sensoe
        telemetry.update(); //Update every loop
        
    } //Ends loop
} //Ends class