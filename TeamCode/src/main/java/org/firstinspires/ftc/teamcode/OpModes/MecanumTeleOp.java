package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Helpers.CameraController;
import org.firstinspires.ftc.teamcode.Helpers.ClawController;
import org.firstinspires.ftc.teamcode.Helpers.DriveController;
import org.firstinspires.ftc.teamcode.Helpers.Toggler;
import org.firstinspires.ftc.teamcode.RoadRunnerFiles.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunnerFiles.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;


@TeleOp
public class MecanumTeleOp extends LinearOpMode {

    private static final double FEET_PER_METER = 3.28084f;

    DcMotorEx frontLeft, frontRight, backLeft, backRight, slideRotatorLeft, slideRotatorRight, slideMotor, liftMotor;
    DriveController driveController;
    SampleMecanumDrive drive;

    Servo leftClaw, rightClaw, clawServo, planeLauncher, planeRotator;
//         port2      port1      port4       port0          port3
    ClawController clawController;

    CameraController cameraController;

    public int autoCalibrateScore(SampleMecanumDrive uhhh, CameraController cameraController, Pose2d pose) {
        AprilTagDetection tag = cameraController.detectAprilTag();

        if(tag == null) {
            return(-1);
        }

        int tagID = tag.id;

        for(int i = 0; i < 5; i++) {
            tag = cameraController.detectAprilTag(tagID);
            Orientation rot = Orientation.getOrientation(tag.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

            TrajectorySequence move = uhhh.trajectorySequenceBuilder(pose)
                    .lineToLinearHeading(new Pose2d(tag.pose.x*FEET_PER_METER+pose.getX()+0.85f,tag.pose.z-2.12,pose.getHeading()+Math.toRadians(rot.firstAngle)))
                    .build();

            uhhh.followTrajectorySequence(move);

//            drive.(tag.pose.x*FEET_PER_METER + 0.85, 0.1f);
//            turnRight(rot.firstAngle, 0.1f);
//            forwards(tag.pose.z - 2.12, 0.1f);

            sleep(500);

        }

        return(tag.id);

    }

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
        drive = new SampleMecanumDrive(hardwareMap);

        planeLauncher = hardwareMap.get(Servo.class, "planeLauncher");
        planeRotator = hardwareMap.get(Servo.class, "planeRotator");

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        clawController = new ClawController(leftClaw, rightClaw, clawServo);

        cameraController = new CameraController();

    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));

        cameraController.initAprilTags(hardwareMap);


        waitForStart();

        planeRotator.setPosition(0.22f);

        while (!isStopRequested()) {

            drive.update();
            Pose2d myPose = drive.getPoseEstimate();

            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", Math.toDegrees(myPose.getHeading()));


            Pose2d poseEstimate = drive.getPoseEstimate();
            if (gamepad1.a) {
                poseEstimate = new Pose2d(poseEstimate.getX(), poseEstimate.getY(),Math.toRadians(0));
                drive.setPoseEstimate(poseEstimate);
            }
            if (gamepad1.x) {
                poseEstimate = new Pose2d(poseEstimate.getX(), poseEstimate.getY(),Math.toRadians(90));
                drive.setPoseEstimate(poseEstimate);
            }
            if (gamepad1.b) {
                poseEstimate = new Pose2d(poseEstimate.getX(), poseEstimate.getY(),Math.toRadians(270));
                drive.setPoseEstimate(poseEstimate);
            }

            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX() * 0.8f * (gamepad1.right_bumper ? 5/4f : 1) * (gamepad1.left_bumper ? 0.25f : 1), //hooly crap question marks in java?? crazy -best
                            input.getY() * 0.8f * (gamepad1.right_bumper ? 5/4f : 1) * (gamepad1.left_bumper ? 0.25f : 1),
                            -gamepad1.right_stick_x * (gamepad1.right_bumper ? 5/4f : 1) * (gamepad1.left_bumper ? 0.25f : 1)
                    )
            );

//            clawController.checkAndToggle(gamepad2.left_bumper, gamepad2.right_bumper);
            clawController.checkAndToggleHold(gamepad2.left_bumper, gamepad2.right_bumper);


            clawController.toggleClawPosition(gamepad2.y);

            if (gamepad2.dpad_up) {
                planeLauncher.setPosition(0);
            } else if (gamepad2.dpad_down) {
                planeLauncher.setPosition(0.3);
            }

            if (gamepad2.a) {
                planeRotator.setPosition(0.22f);
            } else if (gamepad2.b) {
                planeRotator.setPosition(0.1f);
            }

            if (gamepad1.dpad_down) {
                liftMotor.setPower(1);
            } else if (gamepad1.dpad_up) {
                liftMotor.setPower(-1);
            } else {
                liftMotor.setPower(0);
            }

            telemetry.addData("plane rotator pos",planeRotator.getPosition());

            driveController.rotateArm((-gamepad2.right_stick_y/3f));
//            driveController.checkAndToggleRotator(gamepad2.right_stick_button);
            driveController.moveSlide(-gamepad2.left_stick_y);

            if (gamepad1.y) {
                int tagID = autoCalibrateScore(drive, cameraController, drive.getPoseEstimate());
                telemetry.addData("Tag Found for Calibration:", tagID);
            }

            telemetry.addData("Claw Servo:", clawServo.getPosition());
            telemetry.addData("Claw Left:", leftClaw.getPosition());
            telemetry.addData("Claw Right:", rightClaw.getPosition());
            telemetry.addData("Slide Rotator Left:", slideRotatorLeft.getCurrentPosition());
            telemetry.addData("Slide Rotator Right", slideRotatorRight.getCurrentPosition());
            telemetry.addData("Slide Motor:", slideMotor.getCurrentPosition());
            telemetry.addData("Plane Servo:", planeLauncher.getPosition());
            telemetry.addData("Back Left (left odom):", backLeft.getCurrentPosition());
            telemetry.addData("Back Right (right odom):", backRight.getCurrentPosition());
            telemetry.addData("Front Left (back odom):", frontLeft.getCurrentPosition());
            telemetry.update();

        }

    }
}
