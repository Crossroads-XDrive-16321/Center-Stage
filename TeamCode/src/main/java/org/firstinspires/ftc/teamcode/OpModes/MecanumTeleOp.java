package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helpers.ClawController;
import org.firstinspires.ftc.teamcode.Helpers.DriveController;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {

    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    DriveController driveController;

    Servo leftClaw, rightClaw, clawServo;
    ClawController clawController;

    void initialize() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        driveController = new DriveController(frontLeft, backLeft, frontRight, backRight);
        driveController.init();

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

            clawController.checkAndToggle(gamepad2.a);
            clawController.moveClaw(gamepad2.right_trigger - gamepad2.left_trigger);
            clawController.toggleClawPosition(gamepad2.y);


            telemetry.addData("Claw Servo:", clawServo.getPosition());
            telemetry.update();

        }

    }

}
