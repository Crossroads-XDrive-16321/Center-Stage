package org.firstinspires.ftc.teamcode.OpModes.AutoOpModes;

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
public class DAY1AutoOpBlueRight extends LinearOpMode {

    DcMotorEx frontLeft, frontRight, backLeft, backRight, slideRotatorLeft, slideRotatorRight, slideMotor, liftMotor;
    DriveController driveController;

    Servo leftClaw, rightClaw, clawServo, planeLauncher;
    ClawController clawController;

    TfodProcessor tfod;
    VisionPortal visionPortal;

    double driveSpeed = .3;
    double rotateSpeed = .5;

    void initialize() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        slideRotatorLeft = hardwareMap.get(DcMotorEx.class, "slideRotatorLeft");
        slideRotatorRight = hardwareMap.get(DcMotorEx.class, "slideRotatorRight");
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");

        driveController = new DriveController(frontLeft, backLeft, frontRight, backRight, slideRotatorLeft, slideRotatorRight, slideMotor, liftMotor);
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
        tfod.setMinResultConfidence(0.2f);

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
                if (recognition.getConfidence() > maxConfidence) {
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
        driveController.right(1/8f,driveSpeed);
        driveController.turnRight(180,rotateSpeed); //(mech arm forward)
        clawController.setClawLevelPos();
        sleep(250);
        driveController.backwards(15/16f,driveSpeed); //robot center on tile border center
        //adjust how close the bot needs to be depending on arm length

        clawController.setClawLevelPos();
        sleep(250);

        if (loc == 0) {
            driveController.turnLeft(90f,rotateSpeed);
            driveController.left(1/4f, driveSpeed);
            driveController.backwards(1/16f,driveSpeed);
            sleep(250);//place purple pixel on left tape - right claw
            clawController.toggleRightClaw();
            sleep(250);
            clawController.setClawScoringPos();
            driveController.right(1/4f, driveSpeed);
            driveController.forwards(1/16f,driveSpeed);
            driveController.turnRight(90f,rotateSpeed);
        }
        if (loc == 1) {
            driveController.backwards(1/8f,driveSpeed);
            sleep(250);//place purple pixel on mid tape - right claw
            clawController.toggleRightClaw();
            sleep(250);
            clawController.setClawScoringPos();
            driveController.forwards(1/8f,driveSpeed);
        }
        if (loc == 2) {
            driveController.turnRight(90f,rotateSpeed);
            driveController.right(1/4f, driveSpeed);
            driveController.backwards(1/16f,driveSpeed);
            sleep(250);//place purple pixel on right tape - right claw
            clawController.toggleRightClaw();
            sleep(250);
            clawController.setClawScoringPos();
            driveController.left(1/4, driveSpeed);
            driveController.forwards(1/16f,driveSpeed);
            driveController.turnLeft(90f,rotateSpeed);
        }
        driveController.forwards(4/4f,driveSpeed);
        //ends on the border of the two tiles -ideally


        driveController.turnRight(90,rotateSpeed);


        driveController.forwards(17/16f,driveSpeed); //moves to center of tile 3
        driveController.right(15/8f,driveSpeed); //center of left door tile
        driveController.forwards(5/2f,driveSpeed); //border between two tiles left of backstage
        driveController.left(1/8f,driveSpeed); //aligned with center of backstage
        driveController.forwards(3/4f,driveSpeed);

        clawController.setClawLevelPos();
        sleep(500);
        clawController.toggleLeftClaw();
        sleep(500);



    }


}
