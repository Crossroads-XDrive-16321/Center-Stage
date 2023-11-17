package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helpers.ClawController;
import org.firstinspires.ftc.teamcode.Helpers.DriveController;
import org.firstinspires.ftc.teamcode.Helpers.IMUController;
import org.firstinspires.ftc.teamcode.Helpers.Toggler;


@TeleOp
public class MecanumTeleOp extends LinearOpMode {


    DcMotorEx frontLeft, frontRight, backLeft, backRight, slideRotatorLeft, slideRotatorRight, slideMotor;
    DriveController driveController;
//    BNO055IMU imu;
//    IMUController imuController;

    Servo leftClaw, rightClaw, clawServo, planeLauncher;
    ClawController clawController;

    Toggler xButtonToggler = new Toggler();
    Toggler bButtonToggler = new Toggler();
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
    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        while (!isStopRequested()) {

            //Assisted Tele Op Code
            if (aButtonToggler.toggle(gamepad2.a)) { //pressing a automatically moves the arm from picking up pos to scoring pos
                clawController.toggleClawPosition(true);
                driveController.rotateArm(0.5f);
            }
            if (xButtonToggler.toggle(gamepad1.left_bumper)) { //if statement never firing...
                driveController.turnLeft(90, 0.5);
                telemetry.addLine("turning left");
            }
            if (bButtonToggler.toggle(gamepad1.right_bumper)) { //if statement never firing...
                driveController.turnRight(90, 0.5);
                telemetry.addLine("turning right");
            }

            driveController.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 0.8 + (gamepad1.right_trigger / 5) - (gamepad1.left_trigger / 2));




            clawController.checkAndToggle(gamepad2.left_bumper, gamepad2.right_bumper);
            clawController.toggleClawPosition(gamepad2.y);

            if (gamepad2.dpad_up) {
                planeLauncher.setPosition(0);
            } else if (gamepad2.dpad_down) {
                planeLauncher.setPosition(1);
            }

            driveController.rotateArm((gamepad2.right_trigger - gamepad2.left_trigger));
            driveController.moveSlide(-gamepad2.left_stick_y);


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
            telemetry.update();

        }

    }
}
