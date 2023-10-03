package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ClawTesting extends LinearOpMode {


    Servo leftClaw, rightClaw, clawServo;

    void initialize() {

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

    }
    
    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        while(!isStopRequested()) {

            leftClaw.setPosition(leftClaw.getPosition() + (0.02 * gamepad1.left_stick_x));
            rightClaw.setPosition(rightClaw.getPosition() + (0.02 * gamepad1.right_stick_x));
            clawServo.setPosition(clawServo.getPosition() + (0.02 * gamepad1.right_trigger) + (0.02 * gamepad1.left_trigger));


            telemetry.addData("Left Claw Pos:", leftClaw.getPosition());
            telemetry.addData("Right Claw Pos:", rightClaw.getPosition());
            telemetry.addData("Claw Servo Pos:", clawServo.getPosition());

        }

    }


}
