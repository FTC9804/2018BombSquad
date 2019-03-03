package org.firstinspires.ftc.teamcode.Auto.OldAuto;

//Import Statements
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Auto.HelperClasses.TensorFlow;

//Declaration for display on the driver station
//USES OLD ROTATE
@Disabled
@Autonomous(name = "Camera Test")
public class CameraTest extends TensorFlow {

    Recognition lowestGold;
    boolean centerBlock, rightBlock, leftBlock, hit;

    //Main OpMode method
    public void runOpMode() {

        telemetry.addLine("Running Crater Auto 2..."); telemetry.update();

        //Set all boolean of blocks to false
        centerBlock = false;
        leftBlock = false;
        rightBlock = false;

        hit = false; //set hit variable to false

        initAll("Camera Test", "TeleopMain");//Init all motors, servos, and sensors (including gyro imu, Tfod, and vuforia)
        //Automatically transition to TeleopMain when we finish this auto

        waitForStart(); //Wait for us to start the autonomous

        //Drive backwards until we see the goldBlock in the right position
        double drivePow = 0; //Make drivePow variable to adjust drive speed

        lowestGold = getGoldBlock(.1);
        telemetry.addLine("Lowest found!"); telemetry.update();
        while(hit == false) {
            if(lowestGold != null){
                telemetry.addLine("Lowest not null!"); telemetry.update();
                hit = true;
                setBothPower(0); //Stop the motors
            }
            else { //else, lowestGold is null, so we don't see any gold block
                telemetry.addLine("Lowest  null!"); telemetry.update();
                setBothPower(-.25); //Set motor powers to drivePow, so we drive backwards at speed of drivePow
                telemetry.addData("Lowest Gold Xpos: ", "null"); //Output gold pos null state
            }
            lowestGold = getGoldBlock(.1); //Find the lowest gold block
        }

        telemetry.addData("Motor speed: ", drivePow);
        telemetry.update(); //Output current motor speeds

        setBothPower(0); //Stop the motors

        CameraDevice.getInstance().setFlashTorchMode(false);


        rotate(-80,.35,5); //Turn to face the block

        driveWithEncoders(30, .4, 2); //Drive forward until we hit the crater
        setExtenderPower(.25); //Run the extender forward to guarantee we are parked next to the crater
        pause(1); //Keep the extender power for 1 second
        setExtenderPower(0); //Stop the extender motor after 1 second has elapsed

    } //Ends runOpMode method
} //Ends class