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

import java.util.List;

@Autonomous
public class AutoOpRedRight extends LinearOpMode {

    DcMotorEx frontLeft, frontRight, backLeft, backRight, slideRotatorLeft, slideRotatorRight, slideMotor;
    DriveController driveController;

    Servo leftClaw, rightClaw, clawServo, planeLauncher;
    ClawController clawController;

    TfodProcessor tfod;
    VisionPortal visionPortal;

    double driveSpeed = .25;
    double rotateSpeed = .5;

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
                .setModelAssetName("model_20231130_175355.tflite")

                .setModelLabels(new String[]{"blue_prop", "red_prop"})
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(false)
                .setModelInputSize(300)
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

        // CAMERA DETECTING
        int loc = -1;

        while(!isStarted()) {
            // sense location

            List<Recognition> currentRecognitions = tfod.getRecognitions();
            float maxConfidence = 0;
            double x = 0;
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


            telemetry.update();
            sleep(20);
        }


        telemetry.addData("Location:", loc);
        telemetry.update();

        //CAMERA DETECTION PROCESSING

        //loc is where the model found the team prop
        driveController.forwards(1/8f,driveSpeed); //robot center on tile center - TO BE ADJUSTED
        driveController.right(3/32f,driveSpeed);
        driveController.turnRight(180,rotateSpeed); //(mech arm forward)
        clawController.setClawLevelPos();
        sleep(250);
        driveController.backwards(4/4f,driveSpeed); //robot center on tile border center
        //adjust how close the bot needs to be depending on arm length

        clawController.setClawLevelPos();
        sleep(250);

        if (loc == 0) {
            driveController.turnLeft(90f,rotateSpeed);
            driveController.left(1/4f, driveSpeed);
            driveController.backwards(1/16f,driveSpeed);
            sleep(250);//place purple pixel on left tape - left claw
            clawController.toggleRightClaw();
            sleep(250);
            clawController.setClawScoringPos();
            driveController.right(1/4f, driveSpeed);
            driveController.forwards(1/16f,driveSpeed);
            driveController.turnRight(90f,rotateSpeed);
        }
        if (loc == 1) {
            driveController.backwards(1/8f,driveSpeed);
            sleep(250);//place purple pixel on mid tape - left claw
            clawController.toggleRightClaw();
            sleep(250);
            clawController.setClawScoringPos();
            driveController.forwards(1/8f,driveSpeed);
        }
        if (loc == 2) {
            driveController.turnRight(90f,rotateSpeed);
            driveController.right(1/4f, driveSpeed);
            driveController.backwards(1/16f,driveSpeed);
            sleep(250);//place purple pixel on right tape - left claw
            clawController.toggleRightClaw();
            sleep(250);
            clawController.setClawScoringPos();
            driveController.left(1/4, driveSpeed);
            driveController.forwards(1/16f,driveSpeed);
            driveController.turnLeft(90f,rotateSpeed);
        }
        driveController.forwards(3/4f,driveSpeed);
        //ends on the border of the two tiles -ideally




        driveController.turnLeft(90,rotateSpeed);
        driveController.forwards(24/16f,driveSpeed);
        driveController.left(19/32f,driveSpeed);
        //rotate arm and toggle claw

        clawController.setClawLevelPos();
        sleep(500);
        driveController.setArmScoringPos(0.5f);
        driveController.setSlidePos(0.3f, 1f);
        sleep(1000);
        clawController.setClawScoringPos();
        sleep(500);

        //adjust in front of what part of the backboard the arm is
        if (loc == 2) { //right
            driveController.right(1/4f,driveSpeed);
            sleep(500); // drop yellow pixel - right claw
            clawController.toggleLeftClaw();
            sleep(500);
            driveController.backwards(1/4f, driveSpeed);
        }
        if (loc == 1) { //mid
            sleep(500); // drop yellow pixel - right claw
            clawController.toggleLeftClaw();
            sleep(500);
            driveController.backwards(1/4f, driveSpeed);
            driveController.right(1/4f,driveSpeed);
        }
        if (loc == 0) { //left
            //driveController.right(1/4f,driveSpeed);
            sleep(500); // drop yellow pixel - right claw
            clawController.toggleLeftClaw();
            sleep(500);
            driveController.backwards(1/4f, driveSpeed);
            driveController.right(1/2f,driveSpeed); //TODO: double check
        }
        //rotate arm and toggle claw

        driveController.setSlidePos(0, 0.4f);
        clawController.setClawLevelPos();
        driveController.setArmGrabbingPos(0.4f);
        sleep(500);


        driveController.right(7/8f,driveSpeed); //TODO: double check - park
        driveController.forwards(9/8f,driveSpeed);
    }


}
