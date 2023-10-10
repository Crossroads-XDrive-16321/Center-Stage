package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Helpers.DriveController;

// INCREDIBLY USEFUL VIDEO for mecanum wheels https://www.youtube.com/watch?v=gnSW2QpkGXQ

@TeleOp
public class ExampleTeleOp extends LinearOpMode {


    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    DriveController driveController;

    void initialize() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        driveController = new DriveController(frontLeft, backLeft, frontRight, backRight);
        driveController.init();

    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        while(!isStopRequested()) {
            //angle of the direction of the joystick
            driveController.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 0.5 + (gamepad1.right_trigger / 2));

        }

    }

}