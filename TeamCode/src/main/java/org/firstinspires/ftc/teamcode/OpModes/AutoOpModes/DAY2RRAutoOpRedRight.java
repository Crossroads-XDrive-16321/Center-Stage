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
public class DAY2RRAutoOpRedRight extends LinearOpMode {

    DcMotorEx frontLeft, frontRight, backLeft, backRight, slideRotatorLeft, slideRotatorRight, slideMotor;
    DriveController driveController;

    Servo leftClaw, rightClaw, clawServo, planeLauncher;
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

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        clawController = new ClawController(leftClaw, rightClaw, clawServo);

        cameraController = new CameraController();

        drive = new SampleMecanumDrive(hardwareMap);
        startPose = new Pose2d(10,-60, Math.toRadians(90));

        purpL = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(8, () -> {
                    //clawController.setClawLevelPos(); //TODO: yep
                })
                .splineToLinearHeading(new Pose2d(12,-32,Math.toRadians(0)),Math.toRadians(180))
                .build();
        purpM = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(8, () -> {
                    //clawController.setClawLevelPos(); //TODO: yep
                })
                .splineToLinearHeading(new Pose2d(16,-35,Math.toRadians(270)), Math.toRadians(90))
                .build();
        purpR = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(8, () -> {
                    //clawController.setClawLevelPos(); //TODO: yep
                })
                .splineToLinearHeading(new Pose2d(35,-32, Math.toRadians(0)),Math.toRadians(180))
                .build();

        yellowL = drive.trajectorySequenceBuilder(purpL.end())
                .addDisplacementMarker(8, () -> {
                    //clawController.setClawScoringPos(); //TODO: yep
                })
                .lineToConstantHeading(new Vector2d(48, -30))
                .build();
        yellowM = drive.trajectorySequenceBuilder(purpM.end())
                .addDisplacementMarker(8, () -> {
                    //clawController.setClawScoringPos(); //TODO: yep
                })
                .splineToLinearHeading(new Pose2d(48,-34, Math.toRadians(0)),Math.toRadians(0))
                .build();
        yellowR = drive.trajectorySequenceBuilder(purpR.end())
                .addDisplacementMarker(8, () -> {
                    //clawController.setClawScoringPos(); //TODO: yep
                })
                .lineToConstantHeading(new Vector2d(48,-42))
                .build();


    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
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
            sleep(3000);//drop purple pixel
            //clawController.toggleRightClaw(); //TODO: yep
            drive.followTrajectorySequence(yellowL);
        } else if (loc == 1) {
            drive.followTrajectorySequence(purpM);
            sleep(3000);//drop purple pixel
            //clawController.toggleRightClaw(); //TODO: yep
            drive.followTrajectorySequence(yellowM);
        } else {
            drive.followTrajectorySequence(purpR);
            sleep(3000);//drop purple pixel
            //clawController.toggleRightClaw(); //TODO: yep
            drive.followTrajectorySequence(yellowR);
        }

        driveController.setArmScoringPos(.5f);
        driveController.setSlidePos(0.2f,0.3f);
        sleep(3000); //clawController.toggleLeftClaw(); //TODO: yep
        driveController.setSlidePos(0,0.3f);
        driveController.setArmGrabbingPos(.5f);

        park = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setReversed(false)
                .lineToConstantHeading(new Vector2d(46,-60))
                .forward(14)
                .build();

        drive.followTrajectorySequence(park);

//        //loc is where the model found the team prop
//        driveController.forwards(1/8f,driveSpeed); //robot center on tile center - TO BE ADJUSTED
//        driveController.right(1/8f,driveSpeed);
//        driveController.turnRight(180,rotateSpeed); //(mech arm forward)
//        clawController.setClawLevelPos();
//        sleep(250);
//        driveController.backwards(3/4f,driveSpeed); //robot center on tile border center
//        //adjust how close the bot needs to be depending on arm length
//
//        clawController.setClawLevelPos();
//        sleep(250);
//
//        if (loc == 0) {
//            driveController.turnLeft(90f,rotateSpeed);
//            driveController.left(1/4f, driveSpeed);
//            sleep(250);//place purple pixel on left tape - right claw
//            clawController.toggleRightClaw();
//            sleep(250);
//            clawController.setClawScoringPos();
//            driveController.right(1/4f, driveSpeed);
//            driveController.turnRight(90f,rotateSpeed);
//        }
//        if (loc == 1) {
//            driveController.backwards(1/8f,driveSpeed);
//            sleep(250);//place purple pixel on mid tape - right claw
//            clawController.toggleRightClaw();
//            sleep(250);
//            clawController.setClawScoringPos();
//            driveController.forwards(1/8f,driveSpeed);
//        }
//        if (loc == 2) {
//            driveController.turnRight(90f,rotateSpeed);
//            driveController.right(1/4f, driveSpeed);
//            sleep(250);//place purple pixel on right tape - right claw
//            clawController.toggleRightClaw();
//            sleep(250);
//            clawController.setClawScoringPos();
//            driveController.left(1/4, driveSpeed);
//            driveController.turnLeft(90f,rotateSpeed);
//        }
//        driveController.forwards(3/4f,driveSpeed);
//        //ends on the border of the two tiles -ideally
//
//
//
//
//        driveController.turnLeft(90,rotateSpeed);
//        driveController.forwards(24/16f,driveSpeed);
//        driveController.left(29/32f,driveSpeed);
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
//        if (loc == 2) { //right
//            driveController.right(1/4f,driveSpeed);
//            sleep(500); // drop yellow pixel - left claw
//            clawController.toggleLeftClaw();
//            sleep(500);
//            driveController.backwards(1/4f, driveSpeed);
//        }
//        if (loc == 1) { //mid
//            sleep(500); // drop yellow pixel - left claw
//            clawController.toggleLeftClaw();
//            sleep(500);
//            driveController.backwards(1/4f, driveSpeed);
//            driveController.right(1/4f,driveSpeed);
//        }
//        if (loc == 0) { //left
//            driveController.left(1/4f,driveSpeed);
//            sleep(500); // drop yellow pixel - left claw
//            clawController.toggleLeftClaw();
//            sleep(500);
//            driveController.backwards(1/4f, driveSpeed);
//            driveController.right(3/4f,driveSpeed); //TODO: double check
//        }
//        //rotate arm and toggle claw
//
//        driveController.setSlidePos(0, 0.4f);
//        clawController.setClawLevelPos();
//        driveController.setArmGrabbingPos(0.4f);
//        sleep(500);
//
//
//        driveController.right(7/8f,driveSpeed); //TODO: double check - park
//        driveController.forwards(9/8f,driveSpeed);
    }


}
