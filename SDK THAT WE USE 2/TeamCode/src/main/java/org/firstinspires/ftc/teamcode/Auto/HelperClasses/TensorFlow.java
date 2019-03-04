package org.firstinspires.ftc.teamcode.Auto.HelperClasses;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public abstract class TensorFlow extends FunctionsForAuto {
    //VARIABLES
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    //VUFORIA KEY
    static final String VUFORIA_KEY = "AddZzkT/////AAABmSxhRyXVhERymcDDAqXjSLxPBSj7vrOCGnAgNqK2yX+PrnaXklRi9Ww5MMvm0dPFj37E5RC9NxZZ/q3WPN37eLcbqO" +
            "Nig/Dg2hKS3oreDEw03w87bSzQxAZszzxQG8RsiWEi6r0LXZCFaWd6E858A+y3uGR0600w43rOjTnu+itNFvC8rOMfJSg45mTXz5/f3Cr4nguP3WodRL1KHQZoIXU508B00V200" +
            "VOYYcdDc56fgUKX3FQ+WthdaOq9PTRhQYNut39aQBLZRHyfv8u4ynFrgDWuQ9F/J30pIg2zGtv5Zl3hBL8bsRQsQA5+uSihSefyvFoQYLbon6w8ZzppJHTUzTzZ1+yP/8uROrCc2aoz";

    //Vuforia variable declaration
    VuforiaLocalizer vuforia;

    //tfod variable declaration
    TFObjectDetector tfod;

    public void initAll(String name, String chosenOpMode){
        initVuforia(); //Initialze the vuforia camera and vuforia
        telemetry.addLine("Vuforia Initialized");
        telemetry.update(); //Add telemetry so we know where we are

        //If our phone is able to, initialize the Tfod Object Detector, if it is not, print telemetry
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            telemetry.update();
        }
        telemetry.addLine("Tfod initialized");
        telemetry.update(); //Add telemetry so we know where we are

        super.initAll(name, chosenOpMode);
    }
    //Initialize the Vuforia localization engines
    public void initVuforia(){
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    //Initialize the Tfod
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    //Returns the Recognition of the goldBlock that is value of the lowest gold on screen,
    //return null if there is no gold found
    public Recognition getGoldBlock(double waitTime) {
        if (opModeIsActive()) {

            int goldX = -1;
            Recognition goldBlock = null;

            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
            timeOne = this.getRuntime();
            timeTwo = this.getRuntime();
            while (opModeIsActive() && timeOne - timeTwo < waitTime) {
                timeOne = this.getRuntime();
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    pause(.2);
                    if (updatedRecognitions != null) {
                        if(updatedRecognitions.size() > 0) {
                            pause(.2);
                            objectsDetected.setValue(updatedRecognitions.size());telemetry.update();
                            double furthestDown = 720;
                            goldBlock = null;
                            for(Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL) && recognition.getLeft() < furthestDown) {
                                    goldBlock = recognition;
                                    return goldBlock;
                                }
                            }
                        }
                    }
                }
            }
            return goldBlock;
        }
        return null;
    }

    //OLD METHODS

    //Find the position of the gold using if you only see the left two blocks
    public int findPositionTwo() {
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        telemetry.update();
                        if (updatedRecognitions.size() == 2) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getTop();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getTop();
                                } else {
                                    silverMineral2X = (int) recognition.getTop();
                                }
                            }
                            if(goldMineralX == -1){
                                telemetry.addData("Gold Mineral Position", "Right");
                                telemetry.addLine("Cannot see gold Mineral X position");
                                telemetry.update();
                                return 1;
                            }
                            else if(silverMineral2X == -1){
                                if(goldMineralX < silverMineral1X){
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    telemetry.addData("Gold Mineral X Position: ", goldMineralX);
                                    telemetry.update();
                                    return -1;
                                }
                                else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    telemetry.addData("Gold Mineral X Position: ", goldMineralX);
                                    telemetry.update();
                                    return 0;
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        return -1;
    }

    //Returns true if we see the gold in searchTime amount of time, false if we do not
    public boolean seeGold(double searchTime){
        if (opModeIsActive()) {

            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
            double timeOne = this.getRuntime();
            double timeTwo = this.getRuntime();
            while (opModeIsActive() && timeTwo - timeOne < searchTime) {
                timeTwo = this.getRuntime();
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        if(updatedRecognitions.size() > 0) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            for(Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
            if (tfod != null) {
                tfod.shutdown();
            }
        }
        return false;
    }
}
