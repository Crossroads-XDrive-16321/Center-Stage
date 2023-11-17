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
public class AutoOpRedLeft extends LinearOpMode {

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

        // CAMERA DETECTING
        //paste camera code from blue left here

        driveController.forwards(1/8f,.25f); //robot center on tile center - TO BE ADJUSTED
        driveController.right(3/32f,.25f);
        driveController.turnRight(180,.25f); //(mech arm forward)
        driveController.backwards(1f,.25f); //robot center on tile border center

        int test = 0;

        if (test == 0) {
            driveController.backwards(1/8f,.25f);
            driveController.turnLeft(90f,.25f);
            sleep(1000);//TODO: place purple pixel on left tape
            driveController.turnRight(90f,.25f);
            driveController.forwards(1/8f,.25f);
        }
        if (test == 1) {
            driveController.forwards(1/4f,.25f);
            sleep(1000);//TODO: place purple pixel on mid tape
            driveController.backwards(1/4f,.25f);
        }
        if (test == 2) {
            driveController.backwards(1/8f,.25f);
            driveController.turnRight(90f,.25f);
            sleep(1000);//TODO: place purple pixel on left tape
            driveController.turnLeft(90f,.25f);
            driveController.forwards(1/8f,.25f);
        }
        driveController.forwards(1f,.25f);

        driveController.turnLeft(90,.25f);
        driveController.forwards(15/4f,.25f); //TODO: double check
        driveController.left(3/4f,.25f); //TODO: double check

        if (test == 0) {
            driveController.right(3/16f,.25f);
            sleep(1000); //TODO: extend arm, drop yellow pixel
        }
        if (test == 1) { //mid
            sleep(1000);
            driveController.right(3/16f,.25f); //TODO: double check
        }
        if (test == 2) { //right
            driveController.left(3/16f,.25f); //TODO: double check
            sleep(1000); //extend arm, drop yellow pixel
            driveController.right(3/8f,.25f); //TODO: double check
        }

        driveController.right(1f,.25f);


    }


}
