package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Helpers.CameraController;
import org.firstinspires.ftc.teamcode.Helpers.ClawController;
import org.firstinspires.ftc.teamcode.Helpers.DriveController;
import org.firstinspires.ftc.teamcode.Helpers.Toggler;
import org.openftc.apriltag.AprilTagDetection;


@TeleOp
public class MecanumTeleOp extends LinearOpMode {


    DcMotorEx frontLeft, frontRight, backLeft, backRight, slideRotatorLeft, slideRotatorRight, slideMotor;
    DriveController driveController;
//    BNO055IMU imu;
//    IMUController imuController;

    Servo leftClaw, rightClaw, clawServo, planeLauncher;
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

//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imuController = new IMUController(imu, telemetry);
//        imuController.init();

        planeLauncher = hardwareMap.get(Servo.class, "planeLauncher");

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        clawController = new ClawController(leftClaw, rightClaw, clawServo);

        cameraController = new CameraController();
    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        cameraController.initAprilTags(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {

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

            driveController.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 0.8 + (gamepad1.right_trigger / 5) - (gamepad1.left_trigger / 2));




            clawController.checkAndToggle(gamepad2.left_bumper, gamepad2.right_bumper);
            clawController.toggleClawPosition(gamepad2.y);

            if (gamepad2.dpad_up) {
                planeLauncher.setPosition(0);
            } else if (gamepad2.dpad_down) {
                planeLauncher.setPosition(0.3);
            }

            driveController.rotateArm((gamepad2.right_trigger - gamepad2.left_trigger));
            driveController.moveSlide(-gamepad2.left_stick_y);

            if (gamepad1.x) {
                telemetry.addData("Tag Found for Calibration:", driveController.autoCalibrateScore(cameraController));
            }


            AprilTagDetection tag = cameraController.detectAprilTag(5);

            telemetry.addData("Claw Servo:", clawServo.getPosition());
            telemetry.addData("Claw Left:", leftClaw.getPosition());
            telemetry.addData("Claw Right:", rightClaw.getPosition());
            telemetry.addData("Slide Rotator Left:", slideRotatorLeft.getCurrentPosition());
            telemetry.addData("Slide Rotator Right", slideRotatorRight.getCurrentPosition());
            telemetry.addData("Slide Motor:", slideMotor.getCurrentPosition());
            telemetry.addData("Plane Servo:", planeLauncher.getPosition());
            telemetry.addData("Back Left:", backLeft.getCurrentPosition());
            telemetry.addData("Back Right:", backRight.getCurrentPosition());
            telemetry.addData("Front Left:", frontLeft.getCurrentPosition());
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
