package org.firstinspires.ftc.teamcode.Helpers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveController {

    DcMotorEx frontLeft, backLeft, frontRight, backRight, slideRotatorLeft, slideRotatorRight, slideMotor;

//    int tilesToPos = 1050; to be configured

    public DriveController(DcMotorEx frontLeft, DcMotorEx backLeft, DcMotorEx frontRight, DcMotorEx backRight, DcMotorEx slideRotatorLeft, DcMotorEx slideRotatorRight, DcMotorEx slideMotor) {
        this.frontLeft = frontLeft;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.backRight = backRight;
        this.slideRotatorLeft = slideRotatorLeft;
        this.slideRotatorRight = slideRotatorRight;
        this.slideMotor = slideMotor;
    }

    public void init() {

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setTargetPositionTolerance(25);
        frontLeft.setTargetPositionTolerance(25);
        backRight.setTargetPositionTolerance(25);
        backLeft.setTargetPositionTolerance(25);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRotatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRotatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideRotatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRotatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /**
     *
     * @return returns false if any motor is busy, returns true otherwise.
     */

    public void waitForMotors() {
        while(!(!frontLeft.isBusy() && !backLeft.isBusy() && !frontRight.isBusy() && !backRight.isBusy())) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException("Uncaught", e);
            }
        }
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     *
     * @param leftX the x position of the left stick.
     * @param leftY the y position of the left stick.
     * @param rightX the x position of the right stick.
     * @param speedFactor scales the speed of the robot. Should be a double from 0 to 1.
     */

    public void drive(double leftX, double leftY, double rightX, double speedFactor) {

        speedFactor = Math.min(speedFactor, 1);
        speedFactor = Math.max(speedFactor, 0);

        double r = Math.hypot(leftX, leftY);
        double robotAngle = Math.atan2(-leftY, leftX) - Math.PI / 4;

        final double v1;
        final double v2;
        final double v3;
        final double v4;
        if(speedFactor + Math.abs(rightX) > 1) {
            v1 = (r * Math.cos(robotAngle) + rightX) / (speedFactor + rightX);
            v2 = (r * Math.sin(robotAngle) - rightX) / (speedFactor + rightX);
            v3 = (r * Math.sin(robotAngle) + rightX) / (speedFactor + rightX);
            v4 = (r * Math.cos(robotAngle) - rightX) / (speedFactor + rightX);
        } else {
            v1 = r * Math.cos(robotAngle) + rightX;
            v2 = r * Math.sin(robotAngle) - rightX;
            v3 = r * Math.sin(robotAngle) + rightX;
            v4 = r * Math.cos(robotAngle) - rightX;
        }

        frontLeft.setPower(v1 * speedFactor);
        frontRight.setPower(v2 * speedFactor);
        backLeft.setPower(v3 * speedFactor);
        backRight.setPower(v4 * speedFactor);
    }

    public void turn(double leftPower, double rightPower) {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backLeft.setPower(leftPower);
        backRight.setPower(rightPower);
    }

    void setMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        backLeft.setMode(mode);
        frontRight.setMode(mode);
        backRight.setMode(mode);
    }

    public void rotateArm(float power) {
        slideRotatorRight.setTargetPosition(slideRotatorRight.getCurrentPosition() + 10);
        slideRotatorLeft.setTargetPosition(slideRotatorLeft.getCurrentPosition() + 10);

        if(power == 0) {
            slideRotatorLeft.setTargetPosition(slideRotatorLeft.getCurrentPosition());
            slideRotatorRight.setTargetPosition(slideRotatorRight.getCurrentPosition());
        }

        slideRotatorRight.setPower(power);
        slideRotatorLeft.setPower(power);
    }

    public void moveSlide(float power) {
        slideMotor.setPower(power);
    }

}
