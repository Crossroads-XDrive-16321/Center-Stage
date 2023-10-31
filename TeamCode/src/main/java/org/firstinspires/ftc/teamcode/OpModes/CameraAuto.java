package org.firstinspires.ftc.teamcode.OpModes;

import static com.google.blocks.ftcrobotcontroller.util.CurrentGame.TFOD_LABELS;
import static com.google.blocks.ftcrobotcontroller.util.CurrentGame.TFOD_MODEL_ASSET;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Helpers.ClawController;
import org.firstinspires.ftc.teamcode.Helpers.DriveController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;


@Autonomous
public class CameraAuto extends LinearOpMode {

    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    DriveController driveController;

    Servo leftClaw, rightClaw, clawServo;
    ClawController clawController;

    TfodProcessor tfod;
    VisionPortal visionPortal;

    void initialize() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        driveController = new DriveController(frontLeft, backLeft, frontRight, backRight);
        driveController.init();

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        clawController = new ClawController(leftClaw, rightClaw, clawServo);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        initTfod();

        int loc;
        List<Integer> lastLocs = new ArrayList<>();

        while(!isStarted()) {
            // sense location

            List<Recognition> currentRecognitions = tfod.getRecognitions();
            float maxConfidence = 0;
            double x = 320;
            for (Recognition recognition : currentRecognitions) {
                if (recognition.getConfidence() > maxConfidence && recognition.getConfidence() >= 0.8) {
                    x = (recognition.getLeft() + recognition.getRight()) / 2;
                    maxConfidence = recognition.getConfidence();
                }
            }

            if (x < 640 / 3 && x >= 0) {
                telemetry.addLine("OBJECT DETECTED, LEFT");
                loc = 0;
            } else if (x < 2 * 640 / 3 && x >= 640 / 3) {
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

        while(!isStopRequested()) {

            telemetry.addData("Location: ", loc);
            telemetry.update();

        }

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
                .setModelAspectRatio(16/9)

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

}
