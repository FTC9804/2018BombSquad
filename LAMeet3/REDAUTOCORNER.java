//Package declaration
package org.firstinspires.ftc.teamcode;

//Import statements
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by MarcusLeher on 30/12/2017.
 */

@Autonomous(name = "redcorner", group = "VuforiaAuto")
//@Disabled

//Autonomous to score the ball and three blocks from the red corner position
public class REDAUTOCORNER extends FunctionsForAuto {

    public void runOpMode() throws InterruptedException { //runOpMode() method

        configure("red", "corner"); //Configure with parameters red and corner

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

        pause(.05); //pause for .05 seconds


        if (vuMarkReturn.equalsIgnoreCase("left")) //If vuforia reads left
        {
            driveNewIMU(12.1, 5, .3, true, 0); //Drive forwards for 16.9 inches at .35 power at a 0 degree heading with a 5 second timeout
        }
        else
        {
            driveNewIMU(2.6, 5, .3, true, 0); //Drive forwards for 16.9 inches at .35 power at a 0 degree heading with a 5 second timeout
        }

        pause(1);

        if (vuMarkReturn.equalsIgnoreCase("left") || vuMarkReturn.equalsIgnoreCase("center") || vuMarkReturn.equalsIgnoreCase("unknown as answer") || vuMarkReturn.equalsIgnoreCase("novalue")) //If vuforia reads left
        {
            spinMove (125, false, 5, true); //Spin to 125 degrees, while dropping the touch bar, not starting at .3 power with a 5 second timeout, putting the touchServo down

            pause(.1);

            driveNewIMU(9.5, 5, -.3, false, 125);

        }
        else
        {
            spinMove (70, false, 5, true); //Spin to 125 degrees, while dropping the touch bar, not starting at .3 power with a 5 second timeout, putting the touchServo down

            pause(.1);

            driveNewIMU(9.5, 5, -.3, false, 70);
        }


        pause(.1);

        frontPanGrip.setPosition(.258);
        backPanGrip.setPosition(.112);

        pause(1);

        if (vuMarkReturn.equalsIgnoreCase("left") || vuMarkReturn.equalsIgnoreCase("center") || vuMarkReturn.equalsIgnoreCase("unknown as answer") || vuMarkReturn.equalsIgnoreCase("novalue")) //If vuforia reads left
        {
            driveNewIMU(3, 5, .3, true, 125);
        }
        else
        {
            driveNewIMU(3, 5, .3, true, 70);
        }

        pause(.1);

        leftPanSpin.setPosition(.525); //Set leftPanSpin to position .21, as this is the intaking glyph position
        rightPanSpin.setPosition(.586);

        pause(.1);

        if (vuMarkReturn.equalsIgnoreCase("left") || vuMarkReturn.equalsIgnoreCase("center") || vuMarkReturn.equalsIgnoreCase("unknown as answer") || vuMarkReturn.equalsIgnoreCase("novalue")) //If vuforia reads left
        {
            driveNewIMU(7.5, 1.5, -.3, false, 125);
        }
        else
        {
            driveNewIMU(7.5, 1.5, -.3, false, 70);
        }

        pause(.1);

        if (vuMarkReturn.equalsIgnoreCase("left") || vuMarkReturn.equalsIgnoreCase("center") || vuMarkReturn.equalsIgnoreCase("unknown as answer") || vuMarkReturn.equalsIgnoreCase("novalue")) //If vuforia reads left
        {
            driveNewIMU(8, .5, .3, true, 125);
        }
        else
        {
            driveNewIMU(8, .5, .3, true, 70);
        }



//        pause(.05); //pause for .05 seconds
//
//        touch(false, false, true); //Run touch method with parameters false, false, and true, meaning we are running from red corner position
//
//        pause(.05); //Pause for .05 seconds
//
//        if (vuMarkReturn.equalsIgnoreCase("left")) //If vuforia reads left
//        {
//            strafeNewIMU(8, 2.8, .65, -88); //Strafe 8 inches to the right at .65 power with a 2.8 second timeout keeping a -88 degree heading, to return to the center position before collecting glyphs in auto
//        }
//        else if (vuMarkReturn.equalsIgnoreCase("right")) //If vuforia reads right
//        {
//            strafeNewIMU(8, 2.8, -.65, -88); //Strafe 8 inches to the left at .65 power with a 2.8 second timeout keeping a -88 degree heading, to return to the center position before collecting glyphs in auto
//        }
//        else //Else
//        {
//            //Do not strafe as we are centered
//        }
//
//        telemetry.addData("time elapsed", this.getRuntime()-threeGlyphTimeOne); //Telemetry for time elapsed
//        telemetry.update(); //Update telemetry
//
//        pause(.05); //Pause for .05 seconds
//
//        if (this.getRuntime() - threeGlyphTimeOne < 17.5) { //If enough time is left
//
//            spinMove(88, false, 4, false); //Spin move to 88 degrees to position for intaking blocks, not starting at .3 power and not putting the touchServo down with a 4 second timeout
//
//            pause(.05); //Pause for .05 seconds
//
//            if (this.getRuntime() - threeGlyphTimeOne <20.5) { //If enough time is left
//                getBlocks(60); //getBlocks with 60 inch distance
//            }
//
//            pause(.05); //Pause for .05 seconds
//
//            if (this.getRuntime() - threeGlyphTimeOne < 25  ) { //If enough time is left
//                if (sensorB.getDistance(DistanceUnit.CM) < 10 || sensorC.getDistance(DistanceUnit.CM) < 10) { //If we have at least 1 glyph
//                    scoreBlock(84); //score Block at 84 degrees
//                } else { //Else if we do not have glyph
//                    if (this.getRuntime() - threeGlyphTimeOne  < 27.5) { //If enough time is left
//                        driveNewIMU(4.8, 1.5, .3, true, 84); //Drive forward for 4.8 inches at .3 power at 84 degrees with a 1.5 second timeout to secure a spot in the safe zone and make sure we are not touching any glyphs
//                    }
//                }
//            }
//
//
//        }

    }
}
