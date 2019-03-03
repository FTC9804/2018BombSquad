//TELEOPMAIN created on September 8th by Isaac Dienstag

//Package statements
package org.firstinspires.ftc.teamcode.Teleop;

//Import statements
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

//Declaration for display on the driver station
@TeleOp(name = "TeleOp Drive")
public class TeleopDrive extends OpMode {

    //Object Declarations
    DriveTrain DT;


    //Telemetry variables
    int loopCounter;
    double timeOne, timeTwo;

    public void init(){
        //Initialize our objects to the hardwareMap
        DT = new DriveTrain(hardwareMap.dcMotor.get("m2"), hardwareMap.dcMotor.get("m1"), REVERSE, FORWARD);
    }

    public void start() {
        timeOne = this.getRuntime();
    }

    public void loop(){
        loopCounter++;

        //Set DriveTrain powers to left stick and right stick y
        DT.driveFull(gamepad1.left_stick_y, gamepad1.right_stick_y);

        //Telemetry for testing
        timeTwo = this.getRuntime();
        telemetry.addLine("Telemetry:");
        telemetry.addData("LPS", loopCounter/(timeTwo-timeOne));
        telemetry.addData("Left Power", DT.getLeftPower());
        telemetry.addData("Right Power", DT.getRightPower());
        telemetry.update();
    } //Ends loop
} //Ends class