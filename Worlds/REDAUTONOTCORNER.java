//Package declaration
package org.firstinspires.ftc.teamcode;

//Import statements
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by MarcusLeher on 30/12/2017.
 */

@Autonomous(name = "redNOTcorner", group = "VuforiaAuto")
//@Disabled

//Autonomous to score ball and a block from the red non-corner position
public class REDAUTONOTCORNER extends FunctionsForAuto {

    public void runOpMode() throws InterruptedException { //runOpMode() method

        configure("red", "notcorner"); //Configure with parameters red and relicSide

        pause(.05); //pause for .05 seconds

        calibrateGyro(); //Calibrate the gyro

        //Telemetry
        telemetry.addData("calibrated", true);
        telemetry.update();

        pause(.05); //pause for .05 seconds

        introduceAngle(); //Introduce the angle

        pause(.05); //pause for .5 seconds

        waitForStart(); //Wait for start

        pause(.05); //pause for .5 seconds

        //Telemetry
        telemetry.addData("here", true);
        telemetry.update();

        dropFeelerMoveBallOnlyNewRobot(); //Run dropFeelerMoveBallOnlyNewRobot to score the ball

        pause(.05); //pause for .15 seconds

        driveNewIMU(6, 10, .25, true, 0); //Drive forward 6 inches at .25 power keeping a 0 degree heading with a 10 second limit

        touchServo.setPosition(.45);

        pause(.75); //pause for .5 seconds

        if (vuMarkReturn.equalsIgnoreCase("left")) //If vuforia reads left
        {
            spinMove(230, false, 5, true);

            pause(.1);

            frontPanGrip.setPosition(.258);
            backPanGrip.setPosition(.112);

            pause(1);

            leftPanSpin.setPosition(.525); //Set leftPanSpin to position .21, as this is the intaking glyph position
            rightPanSpin.setPosition(.586);

            pause(1);

            driveNewIMU(16.5, 3.2, -.3, false, 230);

        }
        else if (vuMarkReturn.equalsIgnoreCase("right")) //If vuforia reads right
        {
            pivot(-150, 4);

            leftPanSpin.setPosition(.93); //Set leftPanSpin to position .21 to lower thepan
            rightPanSpin.setPosition(.9366);

            pause(.8);

            frontPanGrip.setPosition(.258);
            backPanGrip.setPosition(.112);

            pause(1);

            leftPanSpin.setPosition(.525); //Set leftPanSpin to position .21, as this is the intaking glyph position
            rightPanSpin.setPosition(.586);

            pause(1);

            driveNewIMU(12.5, 2.5, -.3, false, -150);
        }
        else //else
        {
            spinMove(200, false, 5, true);

            pause(.1);

            frontPanGrip.setPosition(.258);
            backPanGrip.setPosition(.112);

            pause(1);

            leftPanSpin.setPosition(.525); //Set leftPanSpin to position .21, as this is the intaking glyph position
            rightPanSpin.setPosition(.586);

            pause(1);

            driveNewIMU(12.5, 2.7, -.3, false, 200);
        }

        pause(.1);


        if (vuMarkReturn.equalsIgnoreCase("left")) //If vuforia reads left
        {
            driveNewIMU(3, 5, .3, true, -230);
        }
        else if (vuMarkReturn.equalsIgnoreCase("right")) //If vuforia reads right
        {
            driveNewIMU(3, 5, .3, true, -150);
        }
        else //else
        {
            driveNewIMU(3, 5, .3, true, 200);
        }




//        telemetry.addData("time elapsed", this.getRuntime()-threeGlyphTimeOne); //Telemetry for time elapsed
//        telemetry.update(); //Update telemetry
//
//        if (this.getRuntime() - threeGlyphTimeOne < 17.5) { //If enough time is left
//
//            spinMove(152, false, 4, false); //Spin move to 152 degrees to position for intaking blocks, not starting at .3 power and not putting the touchServo down with a 4 second timeout
//
//            if (this.getRuntime() - threeGlyphTimeOne < 20.5) { //If enough time is left
//                getBlocks(60); //getBlocks with 60 inch distance
//            }
//
//            pause(.05); //Pause for .05 seconds
//
//            if (this.getRuntime() - threeGlyphTimeOne < 26) { //If enough time is left
//                if (sensorB.getDistance(DistanceUnit.CM) < 10 || sensorC.getDistance(DistanceUnit.CM) < 10) { //If we have at least 1 glyph
//                    scoreBlock(160); //score Block at 160 degrees
//                } else { //Else if we do not have glyph
//                    if (this.getRuntime() - threeGlyphTimeOne < 27.5) { //If enough time is left
//                        driveNewIMU(4.8, 1.5, .3, true, 160);
//                    }
//                }
//            }
//
//        }
//
//
    }

}




