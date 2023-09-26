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

        waitForStart();

        while(!isStopRequested()) {
            //angle of the direction of the joystick
            if(Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1) {
                double speed = Math.sqrt(gamepad1.left_stick_y*gamepad1.left_stick_y + gamepad1.left_stick_x*gamepad1.left_stick_x);
                drive(Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x), speed, gamepad1.right_stick_x);
            } else {
                drive(0, 0, 0);
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

    public void drive(double theta, double speed, double turn){
        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        if(speed + Math.abs(turn) > 1) {
            frontRight.setPower((sin * speed / max - turn) / (speed + turn));
            backLeft.setPower((sin * speed / max + turn) / (speed + turn));
            frontLeft.setPower((cos * speed / max + turn) / (speed + turn));
            backRight.setPower((cos * speed / max - turn) / (speed + turn));
        } else {
            frontRight.setPower((sin * speed / max - turn));
            backLeft.setPower((sin * speed / max + turn));
            frontLeft.setPower((cos * speed / max + turn));
            backRight.setPower((cos * speed / max - turn));
        }

    }


}
