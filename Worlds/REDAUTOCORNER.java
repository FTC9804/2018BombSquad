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
public class REDAUTOCORNERWILD extends FunctionsForAuto {

    boolean left = true;

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


        if (vuMarkReturn.equalsIgnoreCase("left")) // If vuforia reads left
        {
            driveNewIMU(12.1, 5, .27, true, 0); // drive off platform

            frontBarDown(); // lower bar

            pause(.1); // pause

            spinMove(125, false, 5, true); //Spin to 125 degrees, while dropping the touch bar, not starting at .3 power with a 5 second timeout, putting the touchServo down

            pause(.1); // pause

            driveNewIMU(7.5, 2.7, -.3, false, 125); // back away from cryptobox

            pause(.1); // pause

            release(); // drop glyphs

            pause(.7); // wait for them to settle

            lowerPan(); // lower pan

            driveNewIMU(8.5, 2.2, -.3, false, 125); // back into cryptobox scoring glyphs

            pause(.1); // pause

            driveNewIMU(3, 5, .3, true, 125); // drive away from cryptobox

            if ((this.getRuntime() - threeGlyphTimeOne < 17)) { // if there is enough time to get additional glyphs
                if (attackConfirmation.equalsIgnoreCase("left")) { // if most accesible glyph is in the left position
                    spinMove(100, false, 5, false); // turn towards left position

                    pause(.1); // pause

                    driveNewIMU(20, 3, .6, true, 100); // drive toward glyph pile

                    pause(.1); // pause

                    getBlockOne(); // grab the first block

                    pause(.1); // pause

                    driveNewIMU(6, 3, -.4, false, 100); // back away from glyph pile

                    pause(.1); // pause

                    spinMove(70, false, 5, false); // turn towards the second glyph

                    pause(.1); // pause

                    if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) { // if we don't have 2 glyphs
                        getBlockTwo(); // grab second glyph
                    }

                } else if (attackConfirmation.equalsIgnoreCase("right")) { // if most accessible glyph is in the center position
                    spinMove(81, false, 5, false); // spin toward right position

                    pause(.1); // pause

                    driveNewIMU(20, 3, .6, true, 81); // drive toward pile

                    pause(.1); // pause

                    getBlockOne(); // intake the first block

                    pause(.1); // pause

                    driveNewIMU(6, 3, -.4, false, 81); // backup from pile

                    pause(.1); // pause

                    spinMove(95, false, 5, false); // spin towards second glyph

                    pause(.1); // pause

                    if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) { // if we don't already have 2 glyphs
                        getBlockTwo(); // grab second glyph
                    }

                }
                else { // if most accessible glyph in center position

                    spinMove(100, false, 5, false); // spin to center glyph position

                    pause(.1); // pause

                    driveNewIMU(20, 3, .6, true, 100); // drive towards center position

                    pause(.1); // pause

                    getBlockOne(); // get first block

                    pause(.1); // pause

                    driveNewIMU(6, 3, -.4, false, 100); // backup from glyph pile

                    pause(.1); // pause

                    spinMove(70, false, 5, false); // spin toward second glyph

                    pause(.1); // pause

                    if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) { // if we don't already have 2 glyphs
                        getBlockTwo(); // grab second glyph
                    }

                }

            } else if (vuMarkReturn.equalsIgnoreCase("right")) {

                driveNewIMU(33.6, 5, .43, true, 0); // drive off the platform

                frontBarDown(); // lower front bar

                pause(.1);  // pause

                spinMove(55, false, 5, true); //Spin to 125 degrees, while dropping the touch bar, not starting at .3 power with a 5 second timeout, putting the touchServo down

                pause(.1); // pause

                driveNewIMU(5.5, 2.7, -.3, false, 55); // drive toward cryptobox

                release(); // drop blocks

                pause(.7); // wait for them to settle

                lowerPan(); // lower pan

                driveNewIMU(9.5, 2.2, -.3, false, 55); // drive block into cryptobox

                pause(.1); // pause

                driveNewIMU(3, 5, .3, true, 55); // drive away from cryptobox

                pause(.1); // pause

                if (this.getRuntime() - threeGlyphTimeOne < 17) { // if there is time to grab 2 more glyphs
                    if (attackConfirmation.equalsIgnoreCase("left")) { // if the most accessible block is left

                        spinMove(110, false, 5, false); // spin toward center position

                        pause(.1); // pause

                        driveNewIMU(20, 3, .6, true, 110); // drive towards pile

                        pause(.1); // pause

                        getBlockOne(); // grab first block

                        pause(1); // pause

                        driveNewIMU(6, 3, -.4, false, 110); // backup from glyphs

                        pause(.1); // pause

                        spinMove(70, false, 5, false); // turn towards second glyph

                        pause(.1); // pause

                        if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) { // if we don't already have 2 glyphs
                            getBlockTwo(); // get second glyph
                        }

                        driveNewIMU(6, 3, -.4, false, 110); // backup


                    } else if (attackConfirmation.equalsIgnoreCase("right")) {

                        spinMove(70, false, 5, false); // spin toward right position

                        pause(.1); // pause

                        driveNewIMU(20, 3, .6, true, 70); // drive towards pile

                        pause(.1); // pause

                        getBlockOne(); // grab the first block

                        pause(.1); // pause

                        driveNewIMU(6, 3, -.4, false, 70); // backup from pile

                        pause(1); // pause

                        spinMove(80, false, 5, false); // spin toward second block

                        timeOne = this.getRuntime(); // not sure what this does
                        timeTwo = this.getRuntime(); // or this ???????

                        pause(.1);

                        if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) { // if we don't already have 2 glyphs
                            getBlockTwo(); // grab second glyph
                        }

                    } else {
                        spinMove(90, false, 5, false); // spin toward center glyph

                        pause(.1); // pause

                        driveNewIMU(20, 3, .6, true, 90); // drive toward pile

                        pause(.1); // pause

                        getBlockOne(); // intake first block

                        pause(.1); // pause

                        driveNewIMU(6, 3, -.4, false, 90); // back up from pile

                        pause(1); // pause

                        spinMove(15, false, 5, false); // turn toward second glyph

                        timeOne = this.getRuntime(); // not sure what this does
                        timeTwo = this.getRuntime(); // or this

                        pause(.1); // pause

                        if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) { // if we don't already have 2 glyphs
                            getBlockTwo(); // intake second one
                        }

                    }

                }
            }
        }
         else { // if vuforia = center position or nothing

            driveNewIMU(2.6, 5, .27, true, 0); // drive off platform 

            frontBarDown(); // lower front bar 

            pause(.1); // pause 

            spinMove (125, false, 5, true); //Spin to 125 degrees, while dropping the touch bar, not starting at .3 power with a 5 second timeout, putting the touchServo down

            pause(.1); // pause 

            driveNewIMU(7.5, 2.7, -.3, false, 125); // drive back toward cryptobox

            pause(.1); // pause 

            release(); // drop blocks 

            pause(.7); // pause for block to settle 

            lowerPan(); // lower pan 

            driveNewIMU(8.5, 2.2, -.3, false, 125); // push block into cryptobox 

            pause(.1); // pause 

            driveNewIMU(3, 5, .3, true, 125); // drive away from cryptobox 

            pause(.1); // pause 

            if (attackConfirmation.equalsIgnoreCase("left")) { // if most accessible block in left position 
                
                spinMove(100, false, 5, false); // spin towards left block 

                pause(.01); // pause 

                driveNewIMU(20, 3, .6, true, 100); // drive towards blocks

                pause(.01); // pause 

                getBlockOne(); // intake first block 

                pause(.01); // pause 

                driveNewIMU(6, 3, -.4, false, 100); // backup from pile

                pause(.01); // pause 

                spinMove(70, false, 5, false); // turn towards second block 

                pause(.01); // pause

                if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) { // if we don't already have 2 glyphs 
                 
                    getBlockTwo(); // intake second block 
                    
                }

                pause(.01); // pause 

                spinMove(120, false, 5, false); // turn towards cryptobox 

            }
            else if (attackConfirmation.equalsIgnoreCase("right")) { // if most accessible block in right position 
               
                spinMove(66, false, 5, false); // spin towards right block 

                pause(.1); // pause 

                driveNewIMU(20, 3, .6, true, 66); // drive towards blocks 

                pause(.1); // pause 

                getBlockOne(); // intake first block 

                pause(.1); // pause 

                driveNewIMU(6, 3, -.4, false, 66); // backup from pile

                pause(.1); // pause 

                spinMove(100, false, 5, false); // turn toward second block 

                pause(.1); // pause 

                if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) { // if we don't already have 2 glyphs
                   
                    getBlockTwo(); // intake second block
                    
                }

            }
            else { // if most accessible block in center position 
                
                spinMove(85, false, 5, false); // spin towards center block 

                pause(.1); // pause 

                driveNewIMU(20, 3, .6, true, 85); // drive towards blocks 

                pause(.1); // pause 

                getBlockOne(); // intake fist block 

                pause(.1); // pause 

                driveNewIMU(6, 3, -.4, false, 85); // backup from pile 

                pause(.1); // pause 

                spinMove(95 , false, 5, false); // turn towards second block 

                pause(.1); // pause 

                if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) { // if we don't already have 2 glyphs 
                   
                    getBlockTwo(); // intake second block
                
                }

                pause(.1); // pause 

                spinMove(120, false, 5, false); // turn towards cryptobox 

            }
        }

        grab(); // grab blocks 

        pause(.1); // pause 

        if ((this.getRuntime() - threeGlyphTimeOne<24)) { // if there is time 
            driveNewIMU(76, 3, -.5, false, 90); // drive towards cryptobox
        }

        pause(.1); // pause 

        if  (!(sensorB.getDistance(DistanceUnit.CM) < 8) && !(sensorC.getDistance(DistanceUnit.CM) < 8)) { // if we don't have any blocks
            
            driveNewIMU(4.9, 1, .5, true, 90); // drive away from cryptobox 
            
        }
        else
        {
            raisePan(); // raise pan 

            driveNewIMU(5.1, 1, .5, true, 90); // drive away from cryptobox 

            pause(.1); // pause 

            release();

            pause(1); // wait for blocks to settle 

            lowerPan(); // lower pan

            pause(.5); // pause 

            driveNewIMU(10, 3, -.5, false, 70); // back into cryptobox 

            pause(.1); // pause 

            driveNewIMU(6, 3, .5, true, 80); // drive away from cryptobox

            pause(.1); // pause 

            driveNewIMU(10, 3, -.5, false, 90); // knock blocks in

            pause(.1); // pause 

            driveNewIMU(4, 1, .5, true, 90); // drive away

        }
    }

}
