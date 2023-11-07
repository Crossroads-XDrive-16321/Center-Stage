package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helpers.ClawController;
import org.firstinspires.ftc.teamcode.Helpers.DriveController;
import org.firstinspires.ftc.teamcode.Helpers.Toggler;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {

    DcMotorEx frontLeft, frontRight, backLeft, backRight, slideRotatorLeft, slideRotatorRight, slideMotor;
    DriveController driveController;

    Servo leftClaw, rightClaw, clawServo, planeLauncher;
    ClawController clawController;

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


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        while(!isStopRequested()) {
            //angle of the direction of the joystick
            driveController.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 0.8 + (gamepad1.right_trigger / 5) - (gamepad1.left_trigger / 2));

            Toggler xButton = new Toggler();
            Toggler bButton = new Toggler();

            if (xButton.toggle(gamepad1.x)) {
                //rotate 90 ccw
            }
            if (bButton.toggle(gamepad1.b)) {
                //rotate 90 cw
            }


            clawController.checkAndToggle(gamepad2.left_bumper, gamepad2.right_bumper);
            clawController.toggleClawPosition(gamepad2.y);

            if(gamepad2.dpad_up) {
                planeLauncher.setPosition(0);
            } else if(gamepad2.dpad_down) {
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
            telemetry.update();

        }

    }

}
