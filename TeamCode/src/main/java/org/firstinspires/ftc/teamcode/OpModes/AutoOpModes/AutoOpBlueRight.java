package org.firstinspires.ftc.teamcode.OpModes.AutoOpModes;

import static com.google.blocks.ftcrobotcontroller.util.CurrentGame.TFOD_LABELS;
import static com.google.blocks.ftcrobotcontroller.util.CurrentGame.TFOD_MODEL_ASSET;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Helpers.ClawController;
import org.firstinspires.ftc.teamcode.Helpers.DriveController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous
public class AutoOpBlueRight extends LinearOpMode {

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

        // CAMERA DETECTING
        //paste camera code from blue left

        driveController.forwards(1/8f,driveSpeed); //robot center on tile center - TO BE ADJUSTED
        driveController.right(3/32f,driveSpeed);
        driveController.turnRight(180,rotateSpeed); //(mech arm forward)
        driveController.backwards(1f,driveSpeed); //robot center on tile border center
        //adjust how close the bot needs to be depending on arm length
        int test = 0;

        //clawController.toggleClawPosition(true); //TODO: toggle claw

        if (test == 0) {
            driveController.turnLeft(90f,rotateSpeed);
            driveController.backwards(1/8f,driveSpeed);
            sleep(1000);//TODO: place purple pixel on left tape - right claw
            driveController.forwards(1/8f,driveSpeed);
            driveController.turnRight(90f,rotateSpeed);
        }
        if (test == 1) {
            driveController.backwards(1/8f,driveSpeed);
            sleep(1000);//TODO: place purple pixel on mid tape - right claw
            driveController.forwards(1/8f,driveSpeed);
        }
        if (test == 2) {
            driveController.turnRight(90f,rotateSpeed);
            driveController.backwards(1/8f,driveSpeed);
            sleep(1000);//TODO: place purple pixel on left tape - right claw
            driveController.forwards(1/8f,driveSpeed);
            driveController.turnLeft(90f,rotateSpeed);
        }
        driveController.forwards(1f,driveSpeed);
        //ends in the center of the tile -ideally




        driveController.turnRight(90,rotateSpeed);

        //TODO: !!! LITERALLY CHECK EVERYTHING PAST THIS POINT LMAO (all done at home, 0 testing, all numbers completely made up) !!!
        driveController.forwards(1f,driveSpeed); //moves to center of tile 3
        driveController.right(2f,driveSpeed); //center of left door tile
        driveController.forwards(5/2f,driveSpeed); //border between two tiles left of backstage
        driveController.left(1f,driveSpeed); //aligned with center of backstage

        //rotate arm and toggle claw pos

        //this is kinda not made up anymore, but def tweak the numbers yknow...
        if (test == 0) { //left
            driveController.left(3/16f,driveSpeed); //TODO: double check
            sleep(1000); //TODO: drop yellow pixel - left claw
            driveController.right(3/8f,driveSpeed);
        }
        if (test == 1) { //mid
            sleep(1000); //TODO: drop yellow pixel - left claw
            driveController.right(3/16f,driveSpeed); //TODO: double check
        }
        if (test == 2) { //right
            driveController.right(3/16f,driveSpeed); //TODO: double check
            sleep(1000); //TODO: drop yellow pixel - left claw
        }
        //TODO: rotate arm and toggle claw

        driveController.right(1/4f,driveSpeed); //TODO: double check - park

    }


}
