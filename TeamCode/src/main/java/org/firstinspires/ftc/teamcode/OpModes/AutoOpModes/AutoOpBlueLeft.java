package org.firstinspires.ftc.teamcode.OpModes.AutoOpModes;

import static com.google.blocks.ftcrobotcontroller.util.CurrentGame.TFOD_LABELS;
import static com.google.blocks.ftcrobotcontroller.util.CurrentGame.TFOD_MODEL_ASSET;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Helpers.ClawController;
import org.firstinspires.ftc.teamcode.Helpers.DriveController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class AutoOpBlueLeft extends LinearOpMode {

    DcMotorEx frontLeft, frontRight, backLeft, backRight, slideRotatorLeft, slideRotatorRight, slideMotor;
    DriveController driveController;

    Servo leftClaw, rightClaw, clawServo, planeLauncher;
    ClawController clawController;

    TfodProcessor tfod;
    VisionPortal visionPortal;

    double minConfidence = 0.8f;

    void initialize() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        slideRotatorLeft = hardwareMap.get(DcMotorEx.class, "slideRotatorLeft");
        slideRotatorRight = hardwareMap.get(DcMotorEx.class, "slideRotatorRight");
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");

        driveController = new DriveController(frontLeft, backLeft, frontRight, backRight, slideRotatorLeft, slideRotatorRight, slideMotor);
        driveController.init();

        planeLauncher = hardwareMap.get(Servo.class, "planeLauncher");

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        clawController = new ClawController(leftClaw, rightClaw, clawServo);
    }

    private void initTfod() {

        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)

                .setModelLabels(TFOD_LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(false)
                .setModelInputSize(640)
                .setModelAspectRatio(16/9f)

                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 360));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true); // TODO: probably disable this after testing's done

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.8f);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, true);

    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        initTfod();

        waitForStart();

        // CAMERA DETECTING
        int loc;
        List<Integer> lastLocs = new ArrayList<>();

        while(!isStarted()) {
            // sense location

            List<Recognition> currentRecognitions = tfod.getRecognitions();
            float maxConfidence = 0;
            double x = 320;
            for (Recognition recognition : currentRecognitions) {
                if (recognition.getConfidence() > maxConfidence && recognition.getConfidence() >= minConfidence) {
                    x = (recognition.getLeft() + recognition.getRight()) / 2;
                    maxConfidence = recognition.getConfidence();
                }
            }

            if (x < 640 / 3f && x >= 0) {
                telemetry.addLine("OBJECT DETECTED, LEFT");
                loc = 0;
            } else if (x < 2 * 640 / 3f && x >= 640 / 3f) {
                telemetry.addLine("OBJECT DETECTED, MIDDLE");
                loc = 1;
            } else {
                telemetry.addLine("OBJECT DETECTED, RIGHT");
                loc = 2;
            }

            lastLocs.add(loc);
            if(lastLocs.size() > 25) {
                lastLocs.remove(0);
            }
            telemetry.update();
            sleep(20);
        }

        int count0 = 0;
        int count1 = 0;
        int count2 = 0;

        for(int i : lastLocs) {
            switch (i) {
                case 0:
                    count0++;
                case 1:
                    count1++;
                case 2:
                    count2++;
            }
        }

        if(count0 > count1 && count0 > count2) {
            loc = 0;
        } else if(count1 > count0 && count1 > count2) {
            loc = 1;
        } else {
            loc = 2;
        }

//        while(!isStopRequested()) {
//
//            telemetry.addData("Location: ", loc);
//            telemetry.update();
//
//        }

        //loc is where the model found the team prop

        switch (loc) {
            case 0: //left
                //move to left tape
                //place purple pixel on left tape
                //move to spawn
            case 1: //mid
                //move to mid tape
                //place purple pixel on mid tape
                //move to spawn
            case 2: //right
                //move to right tape
                //place purple pixel on right tape
                //move to spawn
        }

        //move from spawn to board

        switch (loc) {
            case 0: //left
                //move to the left
            case 1: //mid
                //nothing
            case 2: //right
                //move to the right
        }
        //drop yellow pixel

        //park


        //temporary parking code that only works if were ONLY parking
        driveController.forwards(0.1, 0.3);
        driveController.left(2.1, 0.4);

    }


}
