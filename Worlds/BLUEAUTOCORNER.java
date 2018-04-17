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

        pause(.05); //pause for .15 seconds

        driveNewIMU(3.75, 10, -.25, false, 0); //Drive backwards 4.75 inches at -.25 power keeping a 0 degree heading with a 10 second limit

        pause(.8); //pause for .8 seconds

        driveNewIMU(9.9, 10, -.25, false, 0);  // drive off platform

        frontBarDown(); // lower bar

        if (vuMarkReturn.equalsIgnoreCase("left")) //If vuforia reads left
        {
            driveNewIMU(22.2, 10, -.45, false, 0); //drive toward left column

            pause(.01); //pause

            spinMove(136, false, 5, true); //spin to line block up with column

            pause(.01); //pause

            driveNewIMU(8.5, 2.7, -.3, false, 136); //Drive toward box

            pause(.01); //pause

            release(); // drop glyphs

            pause(.7); // wait for them to settle

            lowerPan(); // lower pan

            pause(.01); //pause

            driveNewIMU(8.5, 1.6, -.3, false, 131); // back into cryptobox scoring glyphs

            pause(.01); // pause

            driveNewIMU(6.4, 5, .3, true, 139); // drive away from cryptobox

            if ((this.getRuntime() - threeGlyphTimeOne < 17)) { // if there is enough time to get additional glyphs
                if (attackConfirmation.equalsIgnoreCase("left")) { // if most accesible glyph is in the left position
                    spinMove(130, false, 2, false); // turn towards left position

                    pause(.01); // pause

                    driveNewIMU(20, 3, .6, true, 130); // drive toward glyph pile

                    pause(.01); // pause

                    getBlockOne(); // grab the first block

                    pause(.01); // pause

                    driveNewIMU(6, 3, -.4, false, 130); // back away from glyph pile

                    pause(.01); // pause

                    spinMove(120, false, 5, false); // turn towards the second glyph

                    pause(.01); // pause

                    if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) { // if we don't have 2 glyphs
                        getBlockTwo(); // grab second glyph
                    }

                    driveNewIMU(76, 3, -.5, false, 110); // drive towards cryptobox

                    pause(.01); //pause

                    raisePan(); //raise pan to prepare for scoring

                    pause(.01); //pause

                    driveNewIMU(4.9, 1, .5, true, 90); // drive away from cryptobox

                } else if (attackConfirmation.equalsIgnoreCase("right")) { // if most accessible glyph is in the center position
                    spinMove(70, false, 2, false); // spin toward right position

                    pause(.01); // pause

                    driveNewIMU(20, 3, .6, true, 70); // drive toward pile

                    pause(.01); // pause

                    getBlockOne(); // intake the first block

                    pause(.01); // pause

                    driveNewIMU(6, 3, -.4, false, 70); // backup from pile

                    pause(.01); // pause

                    spinMove(80, false, 5, false); // spin towards second glyph

                    pause(.01); // pause

                    if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) { // if we don't already have 2 glyphs
                        getBlockTwo(); // grab second glyph
                    }

                    driveNewIMU(76, 3, -.5, false, 75); // drive towards cryptobox

                    pause(.01); //pause

                    raisePan(); //raise pan to prepare for scoring

                    pause(.01); //pause

                    driveNewIMU(4.9, 1, .5, true, 90); // drive away from cryptobox

                } else { // if most accessible glyph in center position

                    spinMove(85, false, 2, false); // spin to center glyph position

                    pause(.01); // pause

                    driveNewIMU(20, 3, .6, true, 85); // drive towards center position

                    pause(.01); // pause

                    getBlockOne(); // get first block

                    pause(.01); // pause

                    driveNewIMU(6, 3, -.4, false, 85); // backup from glyph pile

                    pause(.01); // pause

                    spinMove(95, false, 5, false); // spin toward second glyph

                    pause(.01); // pause

                    if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) { // if we don't already have 2 glyphs
                        getBlockTwo(); // grab second glyph
                    }

                    driveNewIMU(76, 3, -.5, false, 95); // drive towards cryptobox

                    pause(.01); //pause

                    raisePan(); //raise pan to prepare for scoring

                    pause(.01); //pause

                    driveNewIMU(4.9, 1, .5, true, 90); // drive away from cryptobox

                }

            }
        }
        else if (vuMarkReturn.equalsIgnoreCase("right")) {
            driveNewIMU(7, 5, -.27, false, 0); //Drive toward right column

            pause(.01); //pause

            spinMove(48, false, 5, true);

            pause(.01); //pause

            driveNewIMU(6.5, 2.7, -.3, false, 48);

            pause(.01); //pause

            release(); // drop glyphs

            pause(.7); // wait for them to settle

            lowerPan(); // lower pan

            pause(.01); //pause

            driveNewIMU(10.5, 1.6, -.3, false, 53); // back into cryptobox scoring glyphs

            pause(.01); //pause

            driveNewIMU(6.4, 5, .3, true, 48); // drive away from cryptobox

            if ((this.getRuntime() - threeGlyphTimeOne < 17)) { // if there is enough time to get additional glyphs
                if (attackConfirmation.equalsIgnoreCase("left")) { // if most accesible glyph is in the left position
                    spinMove(120, false, 2, false); // turn towards left position

                    pause(.01); // pause

                    driveNewIMU(20, 3, .6, true, 120); // drive toward glyph pile

                    pause(.01); // pause

                    getBlockOne(); // grab the first block

                    pause(.01); // pause

                    driveNewIMU(6, 3, -.4, false, 120); // back away from glyph pile

                    pause(.01); // pause

                    spinMove(110, false, 5, false); // turn towards the second glyph

                    pause(.01); // pause

                    if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) { // if we don't have 2 glyphs
                        getBlockTwo(); // grab second glyph
                    }

                    driveNewIMU(76, 3, -.5, false, 125); // drive towards cryptobox

                    pause(.01); // pause

                    raisePan(); //raise pan to prepare for scoring

                    pause(.01); // pause

                    driveNewIMU(4.9, 1, .5, true, 90); // drive away from cryptobox

                } else if (attackConfirmation.equalsIgnoreCase("right")) { // if most accessible glyph is in the center position
                    spinMove(60, false, 2, false); // spin to center glyph position

                    pause(.01); // pause

                    driveNewIMU(20, 3, .6, true, 60); // drive towards center position

                    pause(.01); // pause

                    getBlockOne(); // get first block

                    pause(.01); // pause

                    driveNewIMU(6, 3, -.4, false, 60); // backup from glyph pile

                    pause(.01); // pause

                    spinMove(70, false, 5, false); // spin toward second glyph

                    pause(.01); // pause

                    if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) { // if we don't already have 2 glyphs
                        getBlockTwo(); // grab second glyph
                    }

                    driveNewIMU(76, 3, -.5, false, 90); // drive towards cryptobox

                    pause(.01); //pause

                    raisePan(); //raise pan to prepare for scoring

                    pause(.01); //pause

                    driveNewIMU(4.9, 1, .5, true, 90); // drive away from cryptobox

                } else { // if most accessible glyph in center position

                    spinMove(95, false, 2, false); // spin to center glyph position

                    pause(.01); // pause

                    driveNewIMU(20, 3, .6, true, 95); // drive towards center position

                    pause(.01); // pause

                    getBlockOne(); // get first block

                    pause(.01); // pause

                    driveNewIMU(6, 3, -.4, false, 95); // backup from glyph pile

                    pause(.01); // pause

                    spinMove(105, false, 5, false); // spin toward second glyph

                    pause(.01); // pause

                    if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) { // if we don't already have 2 glyphs
                        getBlockTwo(); // grab second glyph
                    }

                    driveNewIMU(76, 3, -.5, false, 105); // drive towards cryptobox

                    pause(.01); //pause

                    raisePan(); //raise pan to prepare for scoring

                    pause(.01); //pause

                    driveNewIMU(4.9, 1, .5, true, 90); // drive away from cryptobox

                }

            }
        }
        else {

            spinMove(48, false, 5, true); //spin move to 48 degrees to allign with cryptobox

            pause(.01); //pause

            release(); //drop blocks

            pause(.7); // wait for them to settle

            lowerPan(); // lower pan

            pause(.01); // pause

            driveNewIMU(12.5, 1.6, -.3, false, 48); // drive block into cryptobox

            pause(.01);  // pause

            driveNewIMU(6.4, 5, .3, true, 48); // drive away from cryptobox

            if ((this.getRuntime() - threeGlyphTimeOne < 17)) { // if there is enough time to get additional glyphs
                if (attackConfirmation.equalsIgnoreCase("left")) { // if most accesible glyph is in the left position
                    spinMove(130, false, 2, false); // turn towards left position

                    pause(.01); // pause

                    driveNewIMU(20, 3, .6, true, 130); // drive toward glyph pile

                    pause(.01); // pause

                    getBlockOne(); // grab the first block

                    pause(.01); // pause

                    driveNewIMU(6, 3, -.4, false, 130); // back away from glyph pile

                    pause(.01); // pause

                    spinMove(120, false, 1, false); // turn towards the second glyph

                    pause(.01); // pause

                    if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) { // if we don't have 2 glyphs
                        getBlockTwo(); // grab second glyph
                    }

                    driveNewIMU(76, 3, -.5, false, 100); // drive towards cryptobox

                    pause(.01); //pause

                    raisePan(); //raise pan to prepare for scoring

                    pause(.01); //pause

                    driveNewIMU(6.4, 1, .5, true, 95); // drive away from cryptobox

                } else if (attackConfirmation.equalsIgnoreCase("right")) { // if most accessible glyph is in the center position
                    spinMove(65, false, 2, false); // spin toward right position

                    pause(.01); // pause

                    driveNewIMU(20, 3, .6, true, 65); // drive toward pile

                    pause(.01); // pause

                    getBlockOne(); // intake the first block

                    pause(.01); // pause

                    driveNewIMU(6, 3, -.4, false, 65); // backup from pile

                    pause(.01); // pause

                    spinMove(85, false, 1, false); // spin towards second glyph

                    pause(.01); // pause

                    if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) { // if we don't already have 2 glyphs
                        getBlockTwo(); // grab second glyph
                    }

                    driveNewIMU(76, 3, -.5, false, 99); // drive towards cryptobox

                    pause(.01); //pause

                    raisePan(); //raise pan to prepare for scoring

                    pause(.01); //pause

                    driveNewIMU(6.4, 1, .5, true, 90); // drive away from cryptobox

                } else { // if most accessible glyph in center position

                    spinMove(110, false, 2, false); // spin to center glyph position

                    pause(.01); // pause

                    driveNewIMU(20, 3, .6, true, 110); // drive towards center position

                    pause(.01); // pause

                    getBlockOne(); // get first block

                    pause(.01); // pause

                    driveNewIMU(6, 3, -.4, false, 110); // backup from glyph pile

                    pause(.01); // pause

                    spinMove(105, false, 1, false); // spin toward second glyph

                    pause(.01); // pause

                    if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) { // if we don't already have 2 glyphs
                        getBlockTwo(); // grab second glyph
                    }

                    driveNewIMU(76, 4, -.3, false, 100); // drive towards cryptobox

                    pause(.01); //pause

                    raisePan(); //raise pan to prepare for scoring

                    pause(.01); //pause

                    release();

                    driveNewIMU(6.4, 1, .5, true, 90); // drive away from cryptobox
                }

                if (this.getRuntime() - threeGlyphTimeOne>24)
                {
                    release(); //release to score block
                }
                else if (sensorB.getDistance(DistanceUnit.CM) < 8 || sensorC.getDistance(DistanceUnit.CM) < 8) {
                    release();

                    pause(.2); // wait for blocks to settle

                    lowerPan(); // lower pan

                    pause(.01); // pause

                    driveNewIMU(10, 3, -.5, false, 90); // back into cryptobox

                    pause(.01); //pause

                    driveNewIMU(5, 3, .5, true, 90); //drive away from cryptobox
                }
            }


        }



    }

}
