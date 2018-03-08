//Package declaration
package org.firstinspires.ftc.teamcode;

//Import statements
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import android.media.MediaPlayer;
import android.media.audiofx.AudioEffect;
import android.media.MediaPlayer.OnBufferingUpdateListener;
import android.media.MediaPlayer.TrackInfo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.*;


/**
 * Created by MarcusLeher on 30/12/2017.
 */

@Autonomous(name = "bluecorner", group = "VuforiaAuto")
//@Disabled

//Autonomous to score the ball and three blocks from the blue corner position
public class BLUEAUTOCORNER extends FunctionsForAuto {

    public void runOpMode() throws InterruptedException { //runOpMode() method

        configure("blue", "corner"); //Configure with parameters blue and corner

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

        driveNewIMU(2.75, 10, -.25, false, 0); //Drive backwards 2.75 inches at -.25 power keeping a 0 degree heading with a 10 second limit, to drive off the stone

        pause(.5); //pause for .75 seconds

        driveNewIMU(11, 10, -.25, false, 0); //Drive backwards 11.7 inches at -.25 power keeping a 0 degree heading with a 10 second limit

        pause(1.3); //Pause for 1.3 seconds

        if (vuMarkReturn.equalsIgnoreCase("left")) //If vuforia reads left
        {
            //Do nothing
        } else if (vuMarkReturn.equalsIgnoreCase("right")) //If vuforia reads right
        {
            driveNewIMU(14, 5, -.32, false, 0); //Drive back 14 inches at -.32 power keeping a 0 degree heading, with a 5 second timeout
        } else //Else
        {
            driveNewIMU(6.4, 5, -.32, false, 0); //Drive back 6.4 inches at -.32 power keeping a 0 degree heading, with a 5 second timeout
        }

        spinMove(-88, false, 5, true); //Spin to -88 degrees not starting at .3 power with a 5 second timeout, putting the touchServo down

        pause(.05); //pause for .05 seconds

        touch(true, true, false); //Run touch method with parameters true, true, and false, meaning we are running from blue corner position

        pause(.05); //Pause for .05 seconds

        if (vuMarkReturn.equalsIgnoreCase("left")) //If vuforia reads left
        {
            strafeNewIMU(8, 2.8, .65, -88); //Strafe 8 inches to the right at .65 power with a 2.8 second timeout keeping a -88 degree heading, to return to the center position before collecting glyphs in auto
        }
        else if (vuMarkReturn.equalsIgnoreCase("right")) //If vuforia reads right
        {
            strafeNewIMU(8, 2.8, -.65, -88); //Strafe 8 inches to the left at .65 power with a 2.8 second timeout keeping a -88 degree heading, to return to the center position before collecting glyphs in auto
        }
        else //Else
        {
            //Do not strafe as we are centered
        }

        telemetry.addData("time elapsed", this.getRuntime()-threeGlyphTimeOne); //Telemetry for time elapsed
        telemetry.update(); //Update telemetry

        pause(.05); //Pause for .05 seconds

        if (this.getRuntime() - threeGlyphTimeOne < 30.5) { //If enough time is left

            spinMove(84, false, 4, false); //Spin move to 84 degrees to position for intaking blocks, not starting at .3 power and not putting the touchServo down with a 4 second timeout

            pause(.05); //Pause for .05 seconds

            if (this.getRuntime() - threeGlyphTimeOne < 40.5) { //If enough time is left
                getBlocks(60); //getBlocks with 60 inch distance
            }

            pause(.05); //Pause for .05 seconds

            if (this.getRuntime() - threeGlyphTimeOne < 50.8) { //If enough time is left
                if (sensorB.getDistance(DistanceUnit.CM) < 10 || sensorC.getDistance(DistanceUnit.CM) < 10) { //If we have at least 1 glyph
                    scoreBlock(84); //score Block at 84 degrees
                } else { //Else if we do not have glyph
                    if (this.getRuntime() - threeGlyphTimeOne < 60.8) { //If enough time is left
                        driveNewIMU(4.8, 1.5, .3, true, 84); //Drive forward for 4.8 inches at .3 power at 84 degrees with a 1.5 second timeout to secure a spot in the safe zone and make sure we are not touching any glyphs
                    }
                }
            }

        }


    }

}
