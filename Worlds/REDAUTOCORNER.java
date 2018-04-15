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


        if (vuMarkReturn.equalsIgnoreCase("left")) //If vuforia reads left
        {
            driveNewIMU(12.1, 5, .27, true, 0); //Drive forwards for 16.9 inches at .35 power at a 0 degree heading with a 5 second timeout
        }
        else if (vuMarkReturn.equalsIgnoreCase("right"))
        {
            driveNewIMU(33.6, 5, .43, true, 0); //Drive forwards for 16.9 inches at .35 power at a 0 degree heading with a 5 second timeout
        }
        else
        {
            driveNewIMU(2.6, 5, .27, true, 0);
        }

        touchServo.setPosition(.45);

        pause(.1);

        if (vuMarkReturn.equalsIgnoreCase("left") || vuMarkReturn.equalsIgnoreCase("center") || vuMarkReturn.equalsIgnoreCase("unknown as answer") || vuMarkReturn.equalsIgnoreCase("novalue")) //If vuforia reads left
        {
            spinMove (125, false, 5, true); //Spin to 125 degrees, while dropping the touch bar, not starting at .3 power with a 5 second timeout, putting the touchServo down

            pause(.1);

            driveNewIMU(7.5, 2.7, -.3, false, 125);

            pause(.1);

            frontPanGrip.setPosition(.258);
            backPanGrip.setPosition(.112);

            pause(.7);

            leftPanSpin.setPosition(.2); //Set leftPanSpin to position .21, as this is the intaking glyph position
            rightPanSpin.setPosition(.17);

            driveNewIMU(8.5, 2.2, -.3, false, 125);

        }
        else
        {
            spinMove (55, false, 5, true); //Spin to 125 degrees, while dropping the touch bar, not starting at .3 power with a 5 second timeout, putting the touchServo down

            pause(.1);

            driveNewIMU(5.5, 2.7, -.3, false, 55);

            frontPanGrip.setPosition(.258);
            backPanGrip.setPosition(.112);

            pause(.7);

            leftPanSpin.setPosition(.2); //Set leftPanSpin to position .21, as this is the intaking glyph position
            rightPanSpin.setPosition(.17);

            driveNewIMU(9.5, 2.2, -.3, false, 55);

        }


        pause(.1);


        if (vuMarkReturn.equalsIgnoreCase("left") || vuMarkReturn.equalsIgnoreCase("center") || vuMarkReturn.equalsIgnoreCase("unknown as answer") || vuMarkReturn.equalsIgnoreCase("novalue") && left) //If vuforia reads left
        {
            driveNewIMU(3, 5, .3, true, 125);
        }
        else
        {
            driveNewIMU(3, 5, .3, true, 55);
        }

        pause(.1);

        if (vuMarkReturn.equalsIgnoreCase("center") || vuMarkReturn.equalsIgnoreCase("unknown as answer") || vuMarkReturn.equalsIgnoreCase("novalue") && (this.getRuntime() - threeGlyphTimeOne<17)) {
            if (attackConfirmation.equalsIgnoreCase("left")) {
                spinMove(100, false, 5, false);

                pause(.01);

                driveNewIMU(20, 3, .6, true, 100);

                pause(.01);

                getBlockOne();

                pause(.01);

                driveNewIMU(6, 3, -.4, false, 100);

                pause(.01);

                spinMove(70, false, 5, false);

                pause(.01);

                if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) {
                    getBlockTwo();
                }

                pause(.01);

                spinMove(120, false, 5, false);

            }
            else if (attackConfirmation.equalsIgnoreCase("right")) {
                spinMove(66, false, 5, false);

                pause(.1);

                driveNewIMU(20, 3, .6, true, 66);

                pause(.1);

                getBlockOne();

                pause(.1);

                driveNewIMU(6, 3, -.4, false, 66);

                pause(.1);

                spinMove(100, false, 5, false);

                pause(.1);

                if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) {
                    getBlockTwo();
                }

            }
            else {
                spinMove(85, false, 5, false);

                pause(.1);

                driveNewIMU(20, 3, .6, true, 85);

                pause(.1);

                getBlockOne();

                pause(.1);

                driveNewIMU(6, 3, -.4, false, 85);

                pause(.1);

                spinMove(95 , false, 5, false);

                pause(.1);

                if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) {
                    getBlockTwo();
                }

                pause(.1);

                spinMove(120, false, 5, false);

            }
        }

        if (vuMarkReturn.equalsIgnoreCase("left")  && (this.getRuntime() - threeGlyphTimeOne<17)) {
            if (attackConfirmation.equalsIgnoreCase("left")) {
                spinMove(100, false, 5, false);

                pause(.1);

                driveNewIMU(20, 3, .6, true, 100);

                pause(.1);

                getBlockOne();

                pause(.1);

                driveNewIMU(6, 3, -.4, false, 100);

                pause(.1);

                spinMove(70, false, 5, false);

                pause(.1);

                if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) {
                    getBlockTwo();
                }

            } else if (attackConfirmation.equalsIgnoreCase("right")) {
                spinMove(81, false, 5, false);

                pause(.1);

                driveNewIMU(20, 3, .6, true, 81);

                pause(.1);

                getBlockOne();

                pause(.1);

                driveNewIMU(6, 3, -.4, false, 81);

                pause(.1);

                spinMove(95, false, 5, false);

                pause(.1);

                if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) {
                    getBlockTwo();
                }

            } else {
                spinMove(100, false, 5, false);

                pause(.1);

                driveNewIMU(20, 3, .6, true, 100);

                pause(.1);

                getBlockOne();

                pause(.1);

                driveNewIMU(6, 3, -.4, false, 100);

                pause(.1);

                spinMove(70, false, 5, false);

                pause(.1);

                if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) {
                    getBlockTwo();
                }

            }
        }

        if (vuMarkReturn.equalsIgnoreCase("right")  && (this.getRuntime() - threeGlyphTimeOne<17)) {
            if (attackConfirmation.equalsIgnoreCase("left")) {
                spinMove(110, false, 5, false);

                pause(.1);

                driveNewIMU(20, 3, .6, true, 110);

                pause(.1);

                getBlockOne();

                pause(1);

                driveNewIMU(6, 3, -.4, false, 110);

                pause(1);

                spinMove(70, false, 5, false);

                pause(1);

                if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) {
                    getBlockTwo();
                }

                driveNewIMU(6, 3, -.4, false, 110);


            }
            else if (attackConfirmation.equalsIgnoreCase("right")) {
                spinMove(70, false, 5, false);

                pause(.1);

                driveNewIMU(20, 3, .6, true, 70);

                pause(.1);

                getBlockOne();

                pause(.1);

                driveNewIMU(6, 3, -.4, false, 70);

                pause(1);

                spinMove(80, false, 5, false);

                timeOne=this.getRuntime();
                timeTwo=this.getRuntime();

                pause(.1);

                if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) {
                    getBlockTwo();
                }

            }

            else {
                spinMove(90, false, 5, false);

                pause(.1);

                driveNewIMU(20, 3, .6, true, 90);

                pause(.1);

                getBlockOne();

                pause(.1);

                driveNewIMU(6, 3, -.4, false, 90);

                pause(1);

                spinMove(15, false, 5, false);

                timeOne=this.getRuntime();
                timeTwo=this.getRuntime();

                pause(.1);

                if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) {
                    getBlockTwo();
                }

            }

        }

        backPanGrip.setPosition(.812);
        frontPanGrip.setPosition(.964);

        pause(.1);

        if (vuMarkReturn.equalsIgnoreCase("right") && (this.getRuntime() - threeGlyphTimeOne<22))
        {
            driveNewIMU(76, 3.8, -.5, false, 80);
        }
        else if (vuMarkReturn.equalsIgnoreCase("left") && (this.getRuntime() - threeGlyphTimeOne<22))
        {
            driveNewIMU(76, 3.8, -.5, false, 100);
        }
        else
        {
            if ((this.getRuntime() - threeGlyphTimeOne<22)) {
                driveNewIMU(76, 3, -.5, false, 90);
            }
        }

        pause(.1);

        if  (!(sensorB.getDistance(DistanceUnit.CM) < 8) && !(sensorC.getDistance(DistanceUnit.CM) < 8)) {
            driveNewIMU(4.9, 1, .5, true, 90);
        }
        else
        {
            leftPanSpin.setPosition(.67);
            rightPanSpin.setPosition(.64);

            driveNewIMU(5.1, 1, .5, true, 90);

            pause(.1);

            frontPanGrip.setPosition(.280);
            backPanGrip.setPosition(.112);

            pause(1);

            leftPanSpin.setPosition(.2);
            rightPanSpin.setPosition(.17);

            pause(.5);

            driveNewIMU(10, 3, -.5, false, 70);

            pause(.1);

            driveNewIMU(6, 3, .5, true, 80);

            pause(.1);

            driveNewIMU(10, 3, -.5, false, 90);

            pause(.1);

            driveNewIMU(4, 1, .5, true, 90);

        }


    }
}
