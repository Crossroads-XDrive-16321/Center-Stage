package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ExampleTeleOp extends LinearOpMode {


    DcMotorEx frontLeft, frontRight, backLeft, backRight;

    void initialize() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        while(!isStopRequested()) {
            //angle of the direction of the joystick
            if(Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1) {
                double speed = Math.sqrt(gamepad1.left_stick_y*gamepad1.left_stick_y + gamepad1.left_stick_x* gamepad1.left_stick_x);
                drive(Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x)-Math.PI/4, speed);
            } else {
                drive(0, 0);
            }

            //quick test
            if (gamepad1.right_bumper){
                move(Math.PI/2, 100);
            }
            if (gamepad1.left_bumper){
                rotate(Math.PI);
            }

            //past method of going about movement

            //frontLeft.setPower(-gamepad1.left_stick_y);
            //backLeft.setPower(-gamepad1.left_stick_y);
            //frontRight.setPower(-gamepad1.right_stick_y);
            //backRight.setPower(-gamepad1.right_stick_y);

        }
        
    }

    public void move(double dir, int dist) {
        ;
    }

    public void rotate(double theta){
        ;
    }

    public void drive(double theta, double speed){
        frontRight.setPower(Math.sin(theta)*speed);
        backLeft.setPower(Math.sin(theta)*speed);
        frontLeft.setPower(Math.cos(theta)*speed);
        backRight.setPower(Math.cos(theta)*speed);
    }


}
