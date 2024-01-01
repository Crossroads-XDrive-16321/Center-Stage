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
import org.openftc.apriltag.AprilTagDetection;


@TeleOp
public class MecanumTeleOp extends LinearOpMode {

    private DigitalChannel redLED0, redLED1, redLED2, redLED3;
    private DigitalChannel greenLED0, greenLED1, greenLED2, greenLED3;



    DcMotorEx frontLeft, frontRight, backLeft, backRight, slideRotatorLeft, slideRotatorRight, slideMotor;
    DriveController driveController;
    SampleMecanumDrive drive;
//    BNO055IMU imu;
//    IMUController imuController;

    Servo leftClaw, rightClaw, clawServo, planeLauncher, planeRotator;
//         port2      port1      port4       port0          port3
    ClawController clawController;

    CameraController cameraController;

    Toggler lbButtonToggler = new Toggler();
    Toggler rbButtonToggler = new Toggler();
    Toggler aButtonToggler = new Toggler();

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
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imuController = new IMUController(imu, telemetry);
//        imuController.init();

        planeLauncher = hardwareMap.get(Servo.class, "planeLauncher");
        planeRotator = hardwareMap.get(Servo.class, "planeRotator");

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        clawController = new ClawController(leftClaw, rightClaw, clawServo);

        cameraController = new CameraController();

//        redLED0 = hardwareMap.get(DigitalChannel.class, "red0"); //expansion0-1
//        greenLED0 = hardwareMap.get(DigitalChannel.class, "green0"); //expansion0-1
//        redLED1 = hardwareMap.get(DigitalChannel.class, "red1"); //expansion2-3
//        greenLED1 = hardwareMap.get(DigitalChannel.class, "green1"); //expansion2-3
//        redLED2 = hardwareMap.get(DigitalChannel.class, "red2"); //control 0-1
//        greenLED2 = hardwareMap.get(DigitalChannel.class, "green2"); //control 0-1
//        redLED3 = hardwareMap.get(DigitalChannel.class, "red3"); //control 2-3
//        greenLED3 = hardwareMap.get(DigitalChannel.class, "green3"); //control 2-3
//
//        redLED0.setMode(DigitalChannel.Mode.OUTPUT);
//        greenLED0.setMode(DigitalChannel.Mode.OUTPUT);
//        redLED1.setMode(DigitalChannel.Mode.OUTPUT);
//        greenLED1.setMode(DigitalChannel.Mode.OUTPUT);
//        redLED2.setMode(DigitalChannel.Mode.OUTPUT);
//        greenLED2.setMode(DigitalChannel.Mode.OUTPUT);
//        redLED3.setMode(DigitalChannel.Mode.OUTPUT);
//        greenLED3.setMode(DigitalChannel.Mode.OUTPUT);
    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));

        cameraController.initAprilTags(hardwareMap);


        waitForStart();

        while (!isStopRequested()) {
            //LIGHTS
//            redLED0.setState(true);
//            redLED1.setState(true);
//            redLED2.setState(true);
//            redLED3.setState(true);
//            greenLED0.setState(false);
//            greenLED1.setState(false);
//            greenLED2.setState(false);
//            greenLED3.setState(false);

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

//
//
//            driveController.drive(gamepad1.left_stick_x*inverseController, gamepad1.left_stick_y*inverseController, gamepad1.right_stick_x, 0.8f + (gamepad1.right_trigger / 5) - (gamepad1.left_trigger / 2));

            //Assisted Tele Op Code
//            if (aButtonToggler.toggle(gamepad2.a)) { //TODO: maybe would be nice if a always brought it to scoring pos and b down to level pos
//                clawController.toggleClawPosition(true);
//                driveController.rotateArm(0.5f);
//                telemetry.addLine("toggling arm");
//            }
//            if (lbButtonToggler.toggle(gamepad1.left_bumper)) {
//                driveController.turnLeft(90, 0.5);
//                telemetry.addLine("turning left");
//            }
//            if (rbButtonToggler.toggle(gamepad1.right_bumper)) {
//                driveController.turnRight(90, 0.5);
//                telemetry.addLine("turning right");
//            }




            clawController.checkAndToggle(gamepad2.left_bumper, gamepad2.right_bumper);
            clawController.toggleClawPosition(gamepad2.y);

            if (gamepad2.dpad_up) {
                planeLauncher.setPosition(0);
            } else if (gamepad2.dpad_down) {
                planeLauncher.setPosition(0.3);
            }

            if (gamepad2.a) {
                ;//planeRotator.setPosition(god knows what);
            } else if (gamepad2.b) {
                ;//planeRotator.setPosition(god knows what else); //TODO: oh god forgot about this
            }

            telemetry.addData("plane rotator pos",planeRotator.getPosition());

            driveController.rotateArm((gamepad2.right_trigger - gamepad2.left_trigger));
            driveController.moveSlide(-gamepad2.left_stick_y);

//            if (gamepad1.x) {
//                telemetry.addData("Tag Found for Calibration:", driveController.autoCalibrateScore(cameraController));
//            }


            AprilTagDetection tag = cameraController.detectAprilTag(5);

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
            telemetry.addData("Front Right:", frontRight.getCurrentPosition());
            if(tag != null) {
                telemetry.addData("Tag Center:", tag.center);
                telemetry.addData("Tag Corners:", tag.corners);
                telemetry.addData("Tag Rot X:", tag.pose.x);
                telemetry.addData("Tag Rot Y:", tag.pose.y);
                telemetry.addData("Tag Rot Z:", tag.pose.z);
                Orientation rot = Orientation.getOrientation(tag.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
                telemetry.addLine(String.format("\nDetected tag ID=%d", tag.id));
                telemetry.addLine(String.format("Translation X: %.2f feet", tag.pose.x*3.28084)); //meter to feet
                telemetry.addLine(String.format("Translation Y: %.2f feet", tag.pose.y*3.28084));
                telemetry.addLine(String.format("Translation Z: %.2f feet", tag.pose.z*3.28084));
                telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
                telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
                telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
            }
            telemetry.update();

        }

    }
}
