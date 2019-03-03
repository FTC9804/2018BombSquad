package org.firstinspires.ftc.teamcode.Auto.OldAuto;

//Import Statements

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Auto.HelperClasses.PixyCam;

//Declaration for display on the driver station
@Disabled
@Autonomous(name = "Crater Auto Pixy")
public class CraterAutoPixy extends PixyCam {

    //Main OpMode method
    public void runOpMode() {

        telemetry.addLine("Running Crater Auto 3..."); telemetry.update();
        Servo sendPowerHere = hardwareMap.servo.get("f1");


        initAll("Crater Auto Pixy", "TeleopMain");//Init all motors, servos, and sensors except Vuforia
        //because we do not use it in this class, also automatically transition to TeleopMain when we finish this auto

        waitForStart();

        //We start hanging, so we call the method dropFromHang(), which pulls out the lock,
        //lowers us down, and unlaches us from the lander, followed by an imu turn to make us
        //parallel to the lander. This method does not run if we chose not to drop.
        dropFromHang();


        //Run our robot to the corner to drop our marker
        driveWithEncoders(17,.4, 2);
        rotate(90,.35,3);
        driveWithEncoders(59,.5, 3);
        rotate(32, .35, 5);
        driveWithEncoders(25,.4,3);

        //Call the method dropMarker(), which extends our intake and runs the intake outwards,
        //which pushes the marker out of our robot. It then retracts the intake and runs the extender
        //at a constant -.2 power, so it doesn't fall down again on the feild, messing us up and damaging the intake
        dropMarker();

        //Drive back and realign with the lander
        driveWithEncoders(30,-.4,3);
        pause(.1);
        rotate(-33, .35, 5);


        turnRunAndHitWithPixy(8);//Drive forward for 8 seconds and look for the block with the pixyCam
        //We also turn to face, and run forward to hit the block once we reach it


        //Run our extender down for one second in order to ensure we are partially parked in the crater
        setExtenderPower(.25);
        pause(1);
        setExtenderPower(0);


    } //Ends runOpMode method
} //Ends class