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

        pause(.05); //pause

        calibrateGyro(); //Calibrate the gyro

        //Telemetry
        telemetry.addData("calibrated", true);
        telemetry.update();

        pause(.05); //pause

        introduceAngle(); //Introduce the angle

        pause(.05); //pause

        waitForStart(); //Wait for start

        pause(.05); //pause

        //Telemetry
        telemetry.addData("here", true);
        telemetry.update();

        dropFeelerMoveBallOnlyNewRobot(); //Run dropFeelerMoveBallOnlyNewRobot to score the ball

        pause(.05); //pause

        driveNewIMU(6, 10, .25, true, 0); //Drive forward

        frontBarDown(); //Lower the front bar to prevent faulty glyphs from intaking

        pause(1); //pause

        if (vuMarkReturn.equalsIgnoreCase("left")) //If vuforia reads left
        {
            spinMove(225, false, 5, true); // spin toward cryptobox

            pause(.01); //pause

            driveNewIMU(8, 3.2, -.3, false, 230); // backup toward cryptobox

            pause(.01); //pause

            release(); // drop blocks

            pause(.7); //wait for them to settle

            lowerPan(); // lower pan

            pause(.01); //pause

            driveNewIMU(8.5, 1.6, -.3, false, 225); // back the glyph in

            pause(.01); //pause

            driveNewIMU(6.1, 5, .3, true, 230); // drive away from cryptobox

            if ((this.getRuntime() - threeGlyphTimeOne < 17)) {
                pause(.01); //pause

                spinMove(136, false, 5, false); // turn towards center line

                pause(.01); //pause

                driveNewIMU(24, 3, .86, true, 125); // drive towards center line

                pause(.01); //pause

                spinMove(170, false, 5, false); // spin towards glyph pile

                pause(.01); //pause

                getBlockOne(); // intake first block

                pause(.01); //pause

                driveNewIMU(6, 3, -.4, false, 180); // backup from pile

                pause(.01); //pause

                spinMove(160, false, 1, false); // spin towards second glyph

                pause(.01); //pause

                if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) { // if we don't already have 2 glyphs

                    getBlockTwo(); // intake second block

                }

                spinMove(150, false, 1, false); // turn towards cryptobox

                grab(); // grab blocks

                driveNewIMU(76, 3.6, -.5, false, 145); // drive into cryptobox


                if (!(sensorB.getDistance(DistanceUnit.CM) < 8) && !(sensorC.getDistance(DistanceUnit.CM) < 8)) // if we didn't pickup any blocks
                {

                    pause(.01); //pause

                    driveNewIMU(4.9, 1, .5, true, 145); // drives away from cryptobox

                } else { // if we do have blocks in our pan

                    raisePan(); // raise pan

                    if (this.getRuntime() - threeGlyphTimeOne > 25) // if there isn't time release glyphs while roating so we throw them into cryptobox
                    {
                        release(); // release glyphs
                    }

                    driveNewIMU(4.9, 1, .5, true, 145); // drive away from cryptobox

                    pause(.2); // wait for pan to reach position

                    release(); // release blocks

                    if (this.getRuntime() - threeGlyphTimeOne < 25) { // if there is time

                        pause(.6); //pause

                        lowerPan(); // lower pan

                        driveNewIMU(10, 1.6, -.5, false, 140); // drive into cryptobox

                        pause(.01); //pause

                        driveNewIMU(6, 3, .5, true, 160); // drive away
                    }

                }
            }

        }
        else if (vuMarkReturn.equalsIgnoreCase("right")) //If vuforia reads right
        {
            raisePan(); // raise pan

            pause(.01); //pause

            pivot(-156, 4); // pivot with left side

            pause(.01); //pause

            release(); // release blocks

            pause(.7); //pause

            lowerPan(); // drop the pan

            pause(.01); //pause

            driveNewIMU(12, 1.6, -.3, false, -156); // ram into cryptobox

            pause(.01); //pause

            driveNewIMU(6.1, 5, .3, true, -156); // drive away from cryptobox

            if ((this.getRuntime() - threeGlyphTimeOne < 17))
            {
                pause(.01); //pause

                spinMove(-80, false, 5, false); // spin towards mid line

                pause(.01); //pause

                driveNewIMU(22, 3, -1, false, -70); // drive towards mid line

                pause(.01); //pause

                spinMove(-180, false, 5, false); // spin towards block pile

                getBlockOne(); // grab first block

                pause(.01); //pause

                driveNewIMU(6, 3, -.4, false, -180); // back up from pile

                pause(.01); //pause

                spinMove(-170, false, 1, false); // turn towards second block

                pause(.01); //pause

                if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) { // if we don't have two blocks

                    getBlockTwo(); // grab second block

                }

                spinMove(-195, false, 1, false); // spin towards the cryptobox

                grab(); // grab blocks

                if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8) || this.getRuntime() - threeGlyphTimeOne > 25) // if we don't have any blocks
                {
                    driveNewIMU(76, 3.6, -.5, false, -195); // ram into cryptobox

                    pause(.01); //pause

                    driveNewIMU(4.9, 1, .5, true, -195); // drive away a little
                } else {

                    driveNewIMU(76, 3.6, -.5, false, -195); // ram into cryptobox

                    pause(.01); //pause

                    raisePan(); // raise pan

                    pause(.01); //pause

                    if (this.getRuntime() - threeGlyphTimeOne > 25) {
                        release(); // drop blocks so we fling them
                    }

                    driveNewIMU(5.1, 1, .5, true, -195); // drive away from cryptobox

                    pause(.01); //pause

                    release(); // release blocks

                    if (this.getRuntime() - threeGlyphTimeOne < 25) { //if there is time

                        pause(.6); //pause

                        lowerPan(); // drop pan

                        pause(.01); //pause

                        driveNewIMU(10, 1.6, -.5, false, -170); // ram into cryptobox

                        pause(.01); //pause

                        driveNewIMU(6, 3, .5, true, -190); // drive away from cryptobox
                    }
                }
            }


        }
        else //else
        {
            spinMove(211, false, 5, true);

            pause(.01);

            release();

            pause(.6);

            lowerPan();

            driveNewIMU(12.5, 1.6, -.3, false, 211);

            pause(.01);

            driveNewIMU(6.1, 5, .3, true, 211);


            if ((this.getRuntime() - threeGlyphTimeOne < 17)) {

                pause(.01);

                spinMove(125, true, 5, false);

                pause(.01);

                driveNewIMU(33, 3, .86, true, 125);

                pause(.01);

                spinMove(170, false, 5, false);

                getBlockOne();

                pause(.01);

                driveNewIMU(6, 3, -.4, false, 180);

                pause(.01);

                spinMove(160, false, 1, false);

                pause(.01);

                if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) {
                    getBlockTwo();
                }

                spinMove(150, false, 1, false);

                grab();

                if (!(sensorB.getDistance(DistanceUnit.CM) < 8) && !(sensorC.getDistance(DistanceUnit.CM) < 8)) {
                    driveNewIMU(76, 5, -.5, false, 145);

                    pause(.01);

                    driveNewIMU(4.9, 1, .5, true, 145);

                } else {

                    driveNewIMU(76, 3.6, -.5, false, 145);

                    pause(.01);

                    raisePan();

                    pause(.01);

                    if (this.getRuntime() - threeGlyphTimeOne > 25) {
                        release();
                    }

                    driveNewIMU(5.1, 1, .5, true, 145);

                    pause(.01);

                    release();

                    if (this.getRuntime() - threeGlyphTimeOne < 25) {

                        pause(.6);

                        lowerPan();

                        driveNewIMU(10, 1, -.5, false, 170);

                        pause(.01);

                        driveNewIMU(6, 3, .5, true, 190);

                    }
                }
            }






        }



    }






}
