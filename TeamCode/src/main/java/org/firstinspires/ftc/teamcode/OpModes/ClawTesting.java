package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helpers.ClawController;
import org.firstinspires.ftc.teamcode.Helpers.Toggler;


// left open = 0.85, closed = 0.47
// right open = 0.25, closed = 0.6

@TeleOp
public class ClawTesting extends LinearOpMode {


    Servo leftClaw, rightClaw, clawServo;
    ClawController clawController;

    void initialize() {

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        clawController = new ClawController(leftClaw, rightClaw, clawServo);

    }
    
    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        rightClaw.setPosition(0.5);
        leftClaw.setPosition(0.5);
        clawServo.setPosition(0.5);

        while(!isStopRequested()) {

            rightClaw.setPosition(rightClaw.getPosition() + (0.001 * gamepad1.right_stick_x));
            leftClaw.setPosition(leftClaw.getPosition() + (0.001 * gamepad1.left_stick_x));
            clawServo.setPosition(clawServo.getPosition() + (0.001 * gamepad1.right_trigger) - (0.001 * gamepad1.left_trigger));

            clawController.checkAndToggle(gamepad2.left_bumper, gamepad2.right_bumper);

            telemetry.addData("Left Claw Pos:", leftClaw.getPosition());
            telemetry.addData("Right Claw Pos:", rightClaw.getPosition());
            telemetry.addData("Claw Servo Pos:", clawServo.getPosition());

            telemetry.update();

        }

    }


}
