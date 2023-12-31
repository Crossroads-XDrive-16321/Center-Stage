package org.firstinspires.ftc.teamcode.OpModes.AutoOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.SpatialMarker;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RoadRunnerFiles.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoadRunnerFiles.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.Helpers.CameraController;
import org.firstinspires.ftc.teamcode.Helpers.ClawController;
import org.firstinspires.ftc.teamcode.Helpers.DriveController;
import org.firstinspires.ftc.teamcode.RoadRunnerFiles.drive.SampleMecanumDrive;

@Autonomous
public class DAY2RRAutoOpBlueLeft extends LinearOpMode {

    DcMotorEx frontLeft, frontRight, backLeft, backRight, slideRotatorLeft, slideRotatorRight, slideMotor;
    DriveController driveController;

    Servo leftClaw, rightClaw, clawServo, planeLauncher;
    ClawController clawController;

    CameraController cameraController;

    double driveSpeed = .25;
    double rotateSpeed = .5;

    private DigitalChannel redLED0, greenLED0;

    SampleMecanumDrive drive;
    Pose2d startPose;

    TrajectorySequence purpL, purpM, purpR;
    TrajectorySequence yellowL, yellowM, yellowR;
    TrajectorySequence park;

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

        cameraController = new CameraController();

        redLED0 = hardwareMap.get(DigitalChannel.class, "red0"); //expansion0-1
        greenLED0 = hardwareMap.get(DigitalChannel.class, "green0"); //expansion0-1
        redLED0.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED0.setMode(DigitalChannel.Mode.OUTPUT);

        drive = new SampleMecanumDrive(hardwareMap);
        startPose = new Pose2d(14,62, Math.toRadians(270));

        purpL = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(34,32, Math.toRadians(0)),Math.toRadians(180))
                .build();
        purpM = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(12,35,Math.toRadians(90)), Math.toRadians(270))
                .build();
        purpR = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(12,32,Math.toRadians(0)),Math.toRadians(180))
                .build();

        yellowL = drive.trajectorySequenceBuilder(purpL.end())
                //.waitSeconds(2) to potentially give the claw time to open? idk - testing required
                .lineToConstantHeading(new Vector2d(46,42))
                .build();
        yellowM = drive.trajectorySequenceBuilder(purpM.end())
                //.waitSeconds(2)
                .splineToLinearHeading(new Pose2d(46,36, Math.toRadians(0)),Math.toRadians(0))
                .build();
        yellowR = drive.trajectorySequenceBuilder(purpR.end())
                //.waitSeconds(2)
                .lineToConstantHeading(new Vector2d(46, 29))
                .build();

    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        redLED0.setState(false);
        greenLED0.setState(false);

        cameraController.initTfod(hardwareMap, 0.2f);

        // CAMERA DETECTING
        int loc = -1;

        while(!isStarted()) {
            // sense location

            loc = cameraController.detectProp();

            telemetry.addLine(String.valueOf(loc));
            telemetry.update();
        }

        loc = 0; //TODO: remove when testing's done lmao


        telemetry.addData("Location:", loc);
        telemetry.update();

        //CAMERA DETECTION PROCESSING

        drive.setPoseEstimate(startPose);

        if (loc == 0) {
            drive.followTrajectorySequence(purpL);
            //drop purple pixel
            //clawController.toggleLeftClaw();
            drive.followTrajectorySequence(yellowL);
            //drop yellow pixel
            //clawController.toggleRightClaw();
        } else if (loc == 1) {
            drive.followTrajectorySequence(purpM);
            //drop purple pixel
            //clawController.toggleLeftClaw();
            drive.followTrajectorySequence(yellowM);
            //drop yellow pixel
            //clawController.toggleRightClaw();
        } else {
            drive.followTrajectorySequence(purpR);
            //drop purple pixel
            //clawController.toggleLeftClaw();
            drive.followTrajectorySequence(purpM);
            //drop yellow pixel
            //clawController.toggleRightClaw();
        }

        park = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setReversed(false)
                .lineToConstantHeading(new Vector2d(46,60))
                .forward(14)
                .build();

        drive.followTrajectorySequence(park);



//        //loc is where the model found the team prop
//        driveController.forwards(1/8f,driveSpeed); //robot center on tile center - TO BE ADJUSTED
//        driveController.right(3/32f,driveSpeed);
//        driveController.turnRight(180,rotateSpeed); //(mech arm forward)
//        clawController.setClawLevelPos();
//        sleep(250);
//        driveController.backwards(7/8f,driveSpeed); //robot center on tile border center
//        //adjust how close the bot needs to be depending on arm length
//
//        clawController.setClawLevelPos();
//        sleep(250);
//
//        if (loc == 0) {
//            driveController.turnLeft(90f,rotateSpeed);
//            driveController.left(1/4f, driveSpeed);
//            driveController.backwards(1/16f,driveSpeed);
//            sleep(250);//place purple pixel on left tape - left claw
//            clawController.toggleLeftClaw();
//            sleep(250);
//            clawController.setClawScoringPos();
//            driveController.right(1/4f, driveSpeed);
//            driveController.forwards(1/16f,driveSpeed);
//            driveController.turnRight(90f,rotateSpeed);
//        }
//        if (loc == 1) {
//            driveController.backwards(1/8f,driveSpeed);
//            sleep(250);//place purple pixel on mid tape - left claw
//            clawController.toggleLeftClaw();
//            sleep(250);
//            clawController.setClawScoringPos();
//            driveController.forwards(1/8f,driveSpeed);
//        }
//        if (loc == 2) {
//            driveController.turnRight(90f,rotateSpeed);
//            driveController.right(1/4f, driveSpeed);
//            driveController.backwards(1/16f,driveSpeed);
//            sleep(250);//place purple pixel on right tape - left claw
//            clawController.toggleLeftClaw();
//            sleep(250);
//            clawController.setClawScoringPos();
//            driveController.left(1/4, driveSpeed);
//            driveController.forwards(1/16f,driveSpeed);
//            driveController.turnLeft(90f,rotateSpeed);
//        }
//        driveController.forwards(3/4f,driveSpeed);
//        //ends on the border of the two tiles -ideally
//
//
//
//
//        driveController.turnRight(90,rotateSpeed);
//        driveController.forwards(24/16f,driveSpeed);
//        driveController.right(19/32f,driveSpeed);
//        //rotate arm and toggle claw
//
//        clawController.setClawLevelPos();
//        sleep(500);
//        driveController.setArmScoringPos(0.5f);
//        driveController.setSlidePos(0.3f, 1f);
//        sleep(1000);
//        clawController.setClawScoringPos();
//        sleep(500);
//
//        //adjust in front of what part of the backboard the arm is
//        if (loc == 0) { //left
//            driveController.left(1/4f,driveSpeed);
//            sleep(500); // drop yellow pixel - right claw
//            clawController.toggleRightClaw();
//            sleep(500);
//            driveController.backwards(1/4f, driveSpeed);
//        }
//        if (loc == 1) { //mid
//            sleep(500); // drop yellow pixel - right claw
//            clawController.toggleRightClaw();
//            sleep(500);
//            driveController.backwards(1/4f, driveSpeed);
//            driveController.left(1/4f,driveSpeed);
//        }
//        if (loc == 2) { //right
//            //driveController.right(1/4f,driveSpeed);
//            sleep(500); // drop yellow pixel - right claw
//            clawController.toggleRightClaw();
//            sleep(500);
//            driveController.backwards(1/4f, driveSpeed);
//            driveController.left(1/2f,driveSpeed); //TODO: double check
//        }
//        //rotate arm and toggle claw
//
//        driveController.setSlidePos(0, 0.4f);
//        clawController.setClawLevelPos();
//        driveController.setArmGrabbingPos(0.4f);
//        sleep(500);
//
//
//        driveController.left(7/8f,driveSpeed); //TODO: double check - park
//        driveController.forwards(9/8f,driveSpeed);


    }


}
