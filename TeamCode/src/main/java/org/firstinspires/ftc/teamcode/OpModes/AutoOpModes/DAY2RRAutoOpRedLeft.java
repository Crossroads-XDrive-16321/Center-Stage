package org.firstinspires.ftc.teamcode.OpModes.AutoOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helpers.CameraController;
import org.firstinspires.ftc.teamcode.Helpers.ClawController;
import org.firstinspires.ftc.teamcode.Helpers.DriveController;
import org.firstinspires.ftc.teamcode.RoadRunnerFiles.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunnerFiles.trajectorysequence.TrajectorySequence;

@Autonomous
public class DAY2RRAutoOpRedLeft extends LinearOpMode {

    DcMotorEx frontLeft, frontRight, backLeft, backRight, slideRotatorLeft, slideRotatorRight, slideMotor;
    DriveController driveController;

    Servo leftClaw, rightClaw, clawServo, planeLauncher, planeRotator;
    ClawController clawController;


    CameraController cameraController;

    private SampleMecanumDrive drive;
    private Pose2d startPose;

    private TrajectorySequence purpL, purpM, purpR;
    private TrajectorySequence yellowL, yellowM, yellowR;
    private TrajectorySequence park;

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
        planeRotator = hardwareMap.get(Servo.class, "planeRotator");

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        clawController = new ClawController(leftClaw, rightClaw, clawServo);

        cameraController = new CameraController();

        drive = new SampleMecanumDrive(hardwareMap);
        startPose = new Pose2d(-38,-64, Math.toRadians(90));
        planeRotator.setPosition(0.24f);

        purpL = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(8, () -> {
                    clawController.setClawLevelPos(); //TODO: yep
                })
                .splineToLinearHeading(new Pose2d(-35,-34,Math.toRadians(0)),Math.toRadians(90))
                .build();
        purpM = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(8, () -> {
                    clawController.setClawLevelPos(); //TODO: yep
                })
                .splineToLinearHeading(new Pose2d(-36,-36,Math.toRadians(270)),Math.toRadians(90))
                .build();
        purpR = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(8, () -> {
                    clawController.setClawLevelPos(); //TODO: yep
                })
                .splineToLinearHeading(new Pose2d(-36,-32,Math.toRadians(180)),Math.toRadians(0))
                .build();

        yellowL = drive.trajectorySequenceBuilder(purpL.end())
                .addDisplacementMarker(8, () -> {
                    clawController.setClawScoringPos(); //TODO: yep
                })
                .strafeTo(new Vector2d(-12,-36))
                .strafeTo(new Vector2d(-12,-16))
                .splineToLinearHeading(new Pose2d(32,-11,Math.toRadians(0)),Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(47,-30+6, Math.toRadians(0)), Math.toRadians(0))
                .build();
        yellowM = drive.trajectorySequenceBuilder(purpM.end())
                .lineToLinearHeading(new Pose2d(-58,-11,Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(32,-11))
                .splineToConstantHeading(new Vector2d(47,-36), Math.toRadians(0))
                .build();
        yellowR = drive.trajectorySequenceBuilder(purpR.end())
                .addDisplacementMarker(8, () -> {
                    clawController.setClawScoringPos(); //TODO: yep
                })
                .splineToLinearHeading(new Pose2d(-58,-11,Math.toRadians(0)),Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-36,-11,Math.toRadians(0)))//.strafeTo(new Vector2d(-36,11))
                .lineToLinearHeading(new Pose2d(32,-11,Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(47,-42+6, Math.toRadians(0)), Math.toRadians(0))
                .build();
    }



    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        cameraController.initTfod(hardwareMap, 0.2f);

        // CAMERA DETECTING
        int loc;

        while(!isStarted()) {
            // sense location

            loc = cameraController.detectProp();

            telemetry.addLine(String.valueOf(loc));
            telemetry.update();
        }

        loc = 1; //TODO: remove when testing's done lmao

        telemetry.addData("Location:", loc);
        telemetry.update();

        //CAMERA DETECTION PROCESSING

        drive.setPoseEstimate(startPose);

        if (loc == 0) {
            drive.followTrajectorySequence(purpL);
            sleep(500);//drop purple pixel
            clawController.toggleLeftClaw(); //TODO: yep
            sleep(100);
            clawController.setClawScoringPos();
            sleep(500);
            drive.followTrajectorySequence(yellowL);
        } else if (loc == 1) {
            drive.followTrajectorySequence(purpM);
            sleep(500);//drop purple pixel
            clawController.toggleLeftClaw(); //TODO: yep
            sleep(100);
            clawController.setClawScoringPos();
            sleep(500);
            drive.followTrajectorySequence(yellowM);
        } else {
            drive.followTrajectorySequence(purpR);
            sleep(500);//drop purple pixel
            clawController.toggleLeftClaw(); //TODO: yep
            sleep(100);
            clawController.setClawScoringPos();
            sleep(500);
            drive.followTrajectorySequence(yellowR);
        }

        driveController.setArmScoringPos(.5f);
        driveController.setSlidePos(0.5f,0.5f);
        sleep(800);
        clawController.toggleRightClaw();
        clawController.toggleLeftClaw();
        sleep(500);
        driveController.setSlidePos(0,0.3f);
        driveController.setArmGrabbingPos(.5f);
        sleep(500);

        park = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setReversed(false)
                .lineToConstantHeading(new Vector2d(46,-6))
                .forward(14)
                .build();

        drive.followTrajectorySequence(park);


//        //loc is where the model found the team prop
//        driveController.forwards(1/8f,driveSpeed); //robot center on tile center - TO BE ADJUSTED
//        driveController.right(3/32f,driveSpeed);
//        driveController.turnRight(180,rotateSpeed); //(mech arm forward)
//        clawController.setClawLevelPos();
//        sleep(250);
//        driveController.backwards(4/4f,driveSpeed); //robot center on tile border center
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
//        driveController.forwards(4/4f,driveSpeed);
//        //ends on the border of the two tiles -ideally
//
//
//        driveController.turnLeft(90,rotateSpeed);
//
//
//        driveController.forwards(17/16f,driveSpeed); //moves to center of tile 3
//        driveController.left(15/8f,driveSpeed); //center of left door tile
//        driveController.forwards(5/2f,driveSpeed); //border between two tiles left of backstage
//        driveController.right(5/4f,driveSpeed); //aligned with center of backstage
//
//        //rotate arm and toggle claw pos
//        clawController.setClawLevelPos();
//        sleep(500);
//        driveController.setArmScoringPos(0.5f);
//        driveController.setSlidePos(0.3f, 1f);
//        sleep(1000);
//        clawController.setClawScoringPos();
//        sleep(500);
//
//        //this is kinda not made up anymore, but def tweak the numbers yknow...
//        if (loc == 2) { //right
//            driveController.right(3/16f,driveSpeed);
//            sleep(500);
//            clawController.toggleRightClaw();
//            sleep(500);
//            driveController.left(3/8f,driveSpeed);
//            driveController.backwards(1/4f,driveSpeed);
//        }
//        if (loc == 1) { //mid
//            sleep(500);
//            clawController.toggleRightClaw();
//            driveController.backwards(1/4f,driveSpeed);
//            driveController.left(3/16f,driveSpeed);
//        }
//        if (loc == 0) { //left
//            driveController.left(3/16f,driveSpeed); //TODO: double check
//            sleep(500);
//            clawController.toggleRightClaw();
//            sleep(500);
//            driveController.backwards(1/4f,driveSpeed);
//            driveController.right(1/2f,driveSpeed);
//        }
//        driveController.setSlidePos(0, 0.4f);
//        clawController.setClawLevelPos();
//        driveController.setArmGrabbingPos(0.4f);
//        sleep(500);
//
//        driveController.left(1/4f,driveSpeed); //TODO: double check - park
//        driveController.forwards(9/8f,driveSpeed);


    }


}
