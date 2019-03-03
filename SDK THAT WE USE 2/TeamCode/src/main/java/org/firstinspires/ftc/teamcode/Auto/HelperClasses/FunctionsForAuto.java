package org.firstinspires.ftc.teamcode.Auto.HelperClasses;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class FunctionsForAuto extends InitForAuto {

    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    public void rotate(int degrees, double power, double time) {
            double leftPower, rightPower;
            // restart imu movement tracking.
            // getAngle() returns + when rotating counter clockwise (left) and - when rotating
            // clockwise (right).
            if (getAngle() > degrees) {   // turn right.
                leftPower = -power;
                rightPower = power;
            } else {  // turn left.
                leftPower = power;
                rightPower = -power;
            }

            // set power to rotate.
            left.setPower(leftPower);
            right.setPower(rightPower);
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();
            if (getAngle() > degrees) {// rotate until turn is completed.
                // On right turn we have to get off zero first.
                while (opModeIsActive() && getAngle() == 0 && timeOne - timeTwo < time) {
                    timeOne = this.getRuntime();
                    timeout.setValue(time - (timeOne - timeTwo));
                    //imuAngle.setValue(getAngle());
                    telemetry.update();
                }
                while (opModeIsActive() && getAngle() > degrees && timeOne - timeTwo < time) {
                    timeOne = this.getRuntime();
                    timeout.setValue(time - (timeOne - timeTwo));
                    //imuAngle.setValue(getAngle());
                    telemetry.update();
                }
            } else    // left turn.
                while (opModeIsActive() && getAngle() < degrees && timeOne - timeTwo < time) {
                    timeOne = this.getRuntime();
                    timeout.setValue(time - (timeOne - timeTwo));
                    //imuAngle.setValue(getAngle());
                    telemetry.update();
                }
            // turn the motors off.
        // reset angle tracking on new heading.
                setBothPower(0);
    }

    //drive forward a certain distance at a speed of power
    public void driveWithEncoders(double distance, double power, double time) {

        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        inches = distance;
        rotations = inches / (Math.PI * WHEEL_DIAMETER);
        counts = ENCODER_CPR * rotations * GEAR_RATIO;

        if(power >= 0) {

            right.setTargetPosition(-(right.getCurrentPosition() + (int) counts));
            left.setTargetPosition(-(left.getCurrentPosition() + (int) counts));

            encoderCounts.setValue(counts);
            telemetry.update();

            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            left.setPower(power);
            right.setPower(power);
        }
        else {
            right.setTargetPosition(right.getCurrentPosition() + (int) counts);
            left.setTargetPosition(left.getCurrentPosition() + (int) counts);

            encoderCounts.setValue(counts);
            telemetry.update();

            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            left.setPower(-power);
            right.setPower(-power);
        }
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();
        while ((timeOne - timeTwo < time) && (opModeIsActive()) && (left.isBusy() || right.isBusy())) {
            timeOne = this.getRuntime();
            encoderCounts.setValue(counts);
            timeout.setValue(time - (timeOne - timeTwo));
            telemetry.update();
            this.idle();
        }

        setBothPower(0);

        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setBothPower(0);
    }

    //Drop from hang and turn towards the corner
    public void dropFromHang(){
        if(shouldWeDrop){ //Only run the dropFromHang method if shouldWeDrop is true
            hanger.setPower(-.8);
            pause(.1);
            swapper.setPosition(1);
            pause(1);
            hanger.setPower(.4);
            pause(2.5);
            hanger.setPower(0);
            driveWithEncoders(4.5, .4, 1);
            rotate(7, .35, 3);
        }
    }

    //Drops the marker in front of our robot and re-intakes the extender
    //A negative power is applied to the extender indefinetely in order
    //to be sure that it does not fall down in front of our robot again
    public void dropMarker(){
        extender.setPower(.25);
        pause(1);
        extender.setPower(0);
        sweeper.setPower(-.85);
        pause(1.2);
        extender.setPower(-.7);
        pause(.3);
        sweeper.setPower(0);
        pause(1);
        extender.setPower(-.2);
    }

    //Wait for time before moving onto the next code
    public void pause(double time) {
        double timeOne = this.getRuntime();
        double timeTwo = this.getRuntime();
        while (timeTwo - timeOne < time && !isStopRequested()) {
            timeTwo = this.getRuntime();
        }
    } //Ends method

    //Telemetry Multithreaded Class
    public class TelemetryThread implements Runnable {
        private Thread t;
        private String threadName;

        public TelemetryThread( String name) {
            threadName = name;
        }

        public void run() {
            while(opModeIsActive() && !isStopRequested() && getRuntime() < 30){
                imuAngle.setValue(getAngle());telemetry.update();
            }
        }

        public void startThread () {
            if (t == null) {
                t = new Thread (this, threadName);
                t.start();
            }
        }
    }
}