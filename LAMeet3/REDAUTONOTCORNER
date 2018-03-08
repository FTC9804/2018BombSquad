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

        pause(.15); //pause for .15 seconds

        driveNewIMU(6, 10, .25, true, 0); //Drive forward 6 inches at .25 power keeping a 0 degree heading with a 10 second limit

        pause(.75); //pause for .5 seconds

        if (vuMarkReturn.equalsIgnoreCase("left")) //If vuforia reads left
        {
            strafeNewIMU(26, 5, -.65, 15); //Strafe to the left for 26 inches at -.65 power keeping a 15 degree heading with a 5 second limit
        } else if (vuMarkReturn.equalsIgnoreCase("right")) //If vuforia reads right
        {
            strafeNewIMU(10.4, 5, -.65, 15); //Strafe to the left for 10.4 inches at -.65 power keeping a 15 degree heading with a 5 second limit
        } else //else
        {
            strafeNewIMU(17.4, 5, -.65, 15); //Strafe to the left for 17.4 inches at -.65 power keeping a 15 degree heading with a 5 second limit
        }

        pause(.025); //pause for .025 seconds

        touch(false, false, false); //Run touch method with parameters false, false, and false, meaning we are running from red not corner position

        pause(.025); //Pause for .025 seconds

        if (vuMarkReturn.equalsIgnoreCase("left")) //If vuforia reads left
        {

        } else if (vuMarkReturn.equalsIgnoreCase("right")) //If vuforia reads right
        {
            strafeNewIMU(16.4, 5, -.65, 13); //Strafe to the left for 16.4 inches at -.65 power keeping a 13 degree heading with a 5 second limit
        } else //else
        {
            strafeNewIMU(10, 5, -.65, 13); //Strafe to the left for 13.2 inches at -.63 power keeping a 13 degree heading with a 5 second limit
        }

        if (this.getRuntime() - threeGlyphTimeOne < 17.5) { //If enough time is left

            spinMove(152, false, 4, false); //Spin move to 152 degrees to position for intaking blocks, not starting at .3 power and not putting the touchServo down with a 4 second timeout

            if (this.getRuntime() - threeGlyphTimeOne < 21.5) { //If enough time is left
                getBlocks(60); //getBlocks with 60 inch distance
            }

            pause(.05); //Pause for .05 seconds

            if (this.getRuntime() - threeGlyphTimeOne < 30.8) { //If enough time is left
                if (sensorB.getDistance(DistanceUnit.CM) < 10 || sensorC.getDistance(DistanceUnit.CM) < 10) { //If we have at least 1 glyph
                    scoreBlock(160); //score Block at 160 degrees
                } else { //Else if we do not have glyph
                    if (this.getRuntime() - threeGlyphTimeOne < 50.8) { //If enough time is left
                        driveNewIMU(4.8, 1.5, .3, true, 160);
                    }
                }
            }

        }


    }

}




