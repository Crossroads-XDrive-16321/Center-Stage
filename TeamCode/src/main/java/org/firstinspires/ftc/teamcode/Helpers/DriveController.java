package org.firstinspires.ftc.teamcode.Helpers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveController {

    DcMotorEx frontLeft, backLeft, frontRight, backRight, slideRotatorLeft, slideRotatorRight, slideMotor;
    int slideRotatorDownPosRight = 0;
    int slideRotatorDownPosLeft = 0;
    int slideMotorDownPos = 0;
    int slideRotatorStoppedPosRight = 0;
    int slideRotatorStoppedPosLeft = 0;
    int slideMotorStoppedPos = 0;

    Toggler slideRotatorToggler = new Toggler();
    Toggler slideMotorToggler = new Toggler();

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

        slideRotatorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        slideRotatorStoppedPosLeft = slideRotatorLeft.getCurrentPosition();
        slideRotatorStoppedPosRight = slideRotatorRight.getCurrentPosition();
        slideMotorStoppedPos = slideMotor.getCurrentPosition();
        slideRotatorDownPosLeft = slideRotatorLeft.getCurrentPosition();
        slideRotatorDownPosRight = slideRotatorRight.getCurrentPosition();
        slideMotorDownPos = slideMotor.getCurrentPosition();


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
            v1 = (r * Math.cos(robotAngle) + rightX) / speedFactor + Math.abs(rightX);
            v2 = (r * Math.sin(robotAngle) - rightX) / speedFactor + Math.abs(rightX);
            v3 = (r * Math.sin(robotAngle) + rightX) / speedFactor + Math.abs(rightX);
            v4 = (r * Math.cos(robotAngle) - rightX) / speedFactor + Math.abs(rightX);
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

        if(power == 0) {
            if(slideRotatorToggler.toggle(true)) {
                slideRotatorStoppedPosLeft = slideRotatorLeft.getCurrentPosition();
                slideRotatorStoppedPosRight = slideRotatorRight.getCurrentPosition();
            }
            slideRotatorRight.setTargetPosition(slideRotatorStoppedPosRight);
            slideRotatorLeft.setTargetPosition(slideRotatorStoppedPosLeft);
        } else {
            slideRotatorToggler.toggle(false);
            slideRotatorRight.setTargetPosition((int) (slideRotatorRight.getCurrentPosition() + (100 * power)));
            slideRotatorLeft.setTargetPosition((int) (slideRotatorLeft.getCurrentPosition() + (100 * power)));
        }


        slideRotatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRotatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(power != 0) {
            slideRotatorRight.setPower(Math.abs(power));
            slideRotatorLeft.setPower(Math.abs(power));
        } else {
            slideRotatorRight.setPower(0.5);
            slideRotatorLeft.setPower(0.5);
        }
    }

    public void moveSlide(float power) {

        if(power == 0) {
            if(slideMotorToggler.toggle(true)) {
                slideMotorStoppedPos = slideMotor.getCurrentPosition();
            }

            slideMotor.setTargetPosition(slideMotorStoppedPos);
        } else {
            slideMotorToggler.toggle(false);
            slideMotor.setTargetPosition((int) (slideMotor.getCurrentPosition() + (100 * power)));
        }

        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(power != 0) {
            slideMotor.setPower(Math.abs(power));
        } else {
            slideMotor.setPower(0.5);
        }
    }

}
