//Package declaration
package org.firstinspires.ftc.teamcode;

//Import statements
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by MarcusLeher on 30/12/2017.
 */

@Autonomous(name = "blueNOTcorner", group = "VuforiaAuto")
//@Disabled

//Autonomous to score ball and a block from the blue non-corner position
public class BLUEAUTONOTCORNER extends FunctionsForAuto {

    public void runOpMode() throws InterruptedException { //runOpMode() method

        configure("blue", "notcorner"); //Configure with parameters blue and relicSide

        pause(.05); //pause for .05 seconds

        calibrateGyro(); //Calibrate the gyro

        //Telemetry
        telemetry.addData("calibrated", true);
        telemetry.update();

        pause(.05); //pause for .05 seconds

        introduceAngle(); //Introduce the angle

        pause(.05); //pause for .05 seconds

        waitForStart(); //Wait for start

        pause(.05); //pause for .05 seconds

        //Telemetry
        telemetry.addData("here", true);
        telemetry.update();

        dropFeelerMoveBallOnlyNewRobot(); //Run dropFeelerMoveBallOnlyNewRobot to score the ball

        pause(.15); //pause for .15 seconds

        driveNewIMU(4.75, 10, -.25, false, 0); //Drive backwards 4.75 inches at -.25 power keeping a 0 degree heading with a 10 second limit

        pause(.8); //pause for .8 seconds

        driveNewIMU(9.2, 10, -.25, false, 0);

        touchServo.setPosition(.45);

        pause(.75); //pause for .4 seconds

        if (vuMarkReturn.equalsIgnoreCase("left")) //If vuforia reads left
        {
            strafeNewIMU(12, 5, .65, 0); //Strafe to the right for 4 inches at .65 power keeping a 178 degree heading with a 5 second limit

            pause(.1);

            spinMove(-52, false, 5, true);

            pause(.5);

            frontPanGrip.setPosition(.258);
            backPanGrip.setPosition(.112);

            pause(1);

            leftPanSpin.setPosition(.525); //Set leftPanSpin to position .21, as this is the intaking glyph position
            rightPanSpin.setPosition(.586);

            pause(1);

            driveNewIMU(16.5, 3.2, -.3, false, -52);

        }
        else if (vuMarkReturn.equalsIgnoreCase("right")) //If vuforia reads right
        {
            spinMove(-60, false, 5, true);

            pause(.5);

            frontPanGrip.setPosition(.258);
            backPanGrip.setPosition(.112);

            pause(1);

            leftPanSpin.setPosition(.525); //Set leftPanSpin to position .21, as this is the intaking glyph position
            rightPanSpin.setPosition(.586);

            pause(1);

            driveNewIMU(16.5, 3.2, -.3, false, -60);
        }
        else //Else
        {
            spinMove(-45, false, 5, true);

            pause(.5);

            frontPanGrip.setPosition(.258);
            backPanGrip.setPosition(.112);

            pause(1);

            leftPanSpin.setPosition(.525); //Set leftPanSpin to position .21, as this is the intaking glyph position
            rightPanSpin.setPosition(.586);

            pause(1);

            driveNewIMU(16.5, 3.2, -.3, false, -45);
        }

        pause(.1); //pause for .1 seconds

        if (vuMarkReturn.equalsIgnoreCase("left")) //If vuforia reads left
        {
            driveNewIMU(3, 5, .3, true, -52);
        }
        else if (vuMarkReturn.equalsIgnoreCase("right")) //If vuforia reads right
        {
            driveNewIMU(3, 5, .3, true, -60);
        }
        else //else
        {
            driveNewIMU(3, 5, .3, true, -45);
        }

    }
}
