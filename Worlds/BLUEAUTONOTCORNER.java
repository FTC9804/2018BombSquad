//Package declaration
package org.firstinspires.ftc.teamcode;

//Import statements
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by MarcusLeher on 30/12/2017.
 */

@Autonomous(name = "blueNOTcorner", group = "VuforiaAuto")
//@Disabled

//Autonomous to score ball and a block from the blue non-corner position
public class BLUEAUTONOTCORNERWILD extends FunctionsForAuto {

    boolean right = true;

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

        pause(.05); //pause

        driveNewIMU(4.75, 10, -.25, false, 0); // drive backward off balancing stone

        pause(.8); //pause

        if  (!vuMarkReturn.equalsIgnoreCase("right")) // vuforia reads right
        {
            driveNewIMU(7.2, 10, -.35, false, 0); // drive backward again

            frontBarDown(); // lower front bar

            pause(.01); //pause

            spinMove(-65, false, 5, true); // spin _____ toward right position

            pause(.01); // pause

            release(); // drop blocks

            pause(.6); // wait for them to settle

            lowerPan(); // lower pan to push

            driveNewIMU(22.5, 2.6, -.34, false, -65); //drive backward toward right column

            pause(.01); //pause

            spinMove(55, false, 3.4, false); // spin _____ toward mid-line

            pause(.01); //pause

            driveNewIMU(12, 5, 1, true, 80); // drive toward mid-line

            pause(.01); //pause

            spinMove(15, true, 5, false); //turn toward glyph pile

            getBlockOne(); // intake the first block

            pause(.01); //pause

            grab(); // grab block

            driveNewIMU(18, 1.9, -.5, false, 0); // backup away from pile

            if (!(sensorB.getDistance(DistanceUnit.CM) < 8) && !(sensorC.getDistance(DistanceUnit.CM) < 8)) // if no blocks picked up
            {
                driveNewIMU(76, 2.8, -.5, false, 30); // drive back into cryptobox

                pause(.01); //pause

                driveNewIMU(4.9, 1, .5, true, 30); // drive forward
            }
            else
            {
                driveNewIMU(76, 2.8, -.5, false, 45); // drive back into cryptobox

                raisePan(); // raise

                driveNewIMU(4.9, 1, .5, true, 0); // drive away from cryptobox

                pause(.2); // pause

                release(); // drop blocks

                if (this.getRuntime() - threeGlyphTimeOne<25) { // if there is time

                    pause(.6); // wait for blocks to settle

                    lowerPan(); // lower pan

                    driveNewIMU(10, 2, -.5, false, 0); // drive back into cryptobox

                    pause(.01); //pause

                    driveNewIMU(3, 2, .5, true, 55); // drive away from cryptobox
                }
            }
        }
        if (vuMarkReturn.equalsIgnoreCase("left")) //If vuforia reads left
        {
            driveNewIMU(5.2, 10, -.35, false, 0); // drive backward additional distance

            frontBarDown(); //lower front bar

            pause(.01); //pause for .01 seconds

            strafeNewIMU(12, 5, .65, 0); //Strafe to the right for 4 inches at .65 power keeping a 178 degree heading with a 5 second limit

            pause(.01);

            spinMove(-52, false, 5, true);

            pause(.01);

            release();

            pause(.6);

            lowerPan();

            driveNewIMU(16.5, 1.6, -.3, false, -52);

            pause(.01); //pause for .1 seconds

            driveNewIMU(6.4, 5, .3, true, -52);

            spinMove(55, false, 3.4, false);

            pause(.01);

            driveNewIMU(35, 5, 1, true, 80);

            pause(.01);

            spinMove(15, true, 5, false);

            getBlockOne();

            pause(.01);

            grab();

            driveNewIMU(18, 1.9, -.5, false, 0);

            if (!(sensorB.getDistance(DistanceUnit.CM) < 8) && !(sensorC.getDistance(DistanceUnit.CM) < 8))
            {
                driveNewIMU(76, 2.8, -.5, false, 30);

                pause(.01);

                driveNewIMU(4.9, 1, .5, true, 30);
            }
            else {
                driveNewIMU(76, 2.8, -.5, false, 19);

                raisePan();

                driveNewIMU(4.9, 1, .5, true, 19);

                pause(.2);

                release();

                if (this.getRuntime() - threeGlyphTimeOne < 25) {
                    pause(.6);

                    lowerPan();

                    driveNewIMU(10, 2, -.5, false, 19);

                    pause(.01);

                    driveNewIMU(3, 2, .5, true, 26);

                }
            }
        }
        else //aka center
        {
            driveNewIMU(5.2, 10, -.35, false, 0); // drive backward away from stone

            frontBarDown(); // lower front bar

            pause(.01); // pause

            spinMove(-45, false, 5, true); // spin _____ toward cryptobox

            pause(.01); // pause

            release(); // release glyphs

            pause(.6); // wait for them to settle

            lowerPan(); // lower pan

            driveNewIMU(16.5, 1.6, -.3, false, -45); // drive into cryptobox

            pause(.01); //pause

            driveNewIMU(6.4, 5, .3, true, -45); //drive away from cryptobox

            spinMove(55, false, 3.4, false); // spin toward center line

            pause(.01); // pause

            driveNewIMU(27.8, 5, 1, true, 80); // drive toward center line

            pause(.01); // pause

            spinMove(15, true, 5, false); // spin toward glyph pile

            getBlockOne(); // get first glyph

            pause(.01); // pause

            grab(); // grab glyph

            driveNewIMU(18, 1.9, -.5, false, 0); // drive backward from pile

            if (!(sensorB.getDistance(DistanceUnit.CM) < 8) && !(sensorC.getDistance(DistanceUnit.CM) < 8)) // if no glyphs in grabbers
            {
                driveNewIMU(76, 2.8, -.5, false, 30); // drive back into the cryptobox

                pause(.01); // pause

                driveNewIMU(4.9, 1, .5, true, 30); // drive away from cryptobox
            }
            else { // we do have at least one block

                driveNewIMU(76, 2.8, -.5, false, 14); // drive toward cryptobox

                raisePan(); // raise pan

                driveNewIMU(4.9, 1, .5, true, 14); // drive away from cryptobox

                pause(.2); // pause

                release(); // drop blocks

                if (this.getRuntime() - threeGlyphTimeOne < 25) { // if there is time left

                    pause(.6); // wait for blocks to settle

                    lowerPan(); // drop pan

                    driveNewIMU(10, 2, -.5, false, 14); // drive into cryptobox

                    pause(.01); // pause

                    driveNewIMU(3, 2, .5, true, 7); //drive away from cryptobox

                }
            }
        }
    }
}
