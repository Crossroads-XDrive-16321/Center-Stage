package org.firstinspires.ftc.teamcode.Helpers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class DriveController {

    DcMotorEx frontLeft, backLeft, frontRight, backRight, slideRotatorLeft, slideRotatorRight, slideMotor, liftMotor;
    int slideRotatorDownPosRight = 0;
    int slideRotatorDownPosLeft = 0;
    int slideMotorDownPos = 0;
    int slideRotatorStoppedPosRight = 0; // -360 from down to max
    int slideRotatorStoppedPosLeft = 0;
    int slideMotorStoppedPos = 0;

    int liftMotorDownPos = 0;

    float slidePosToArmLengthMM = 1; // TODO: Calibrate
    float armLengthConstant = 200; // TODO: Measure (in mm)

    float ArmRotatorPosToArmAngle = 1; // TODO: Calibrate

    float stringLengthToStringLifterPos = 1; // TODO: Calibrate

    double currentStringLength = 100; // TODO: Measure (in mm)

    boolean slideRotatorDown = true;
    double tilesToPos = 1087.5625;
    double degreesToPos = 9.39;

    double tilesToPosFR = 901.5714;
    double tilesToPosFL = 1022.0;
    double tilesToPosBR = 1033.8571;
    double tilesToPosBL = 1035.7143;

    double degreesToPosFR = 7.888;
    double degreesToPosFL = 10.088;
    double degreesToPosBR = 9.287;
    double degreesToPosBL = 9.107;

    double FEET_PER_METER = 3.28084;

    Toggler slideRotatorToggler = new Toggler();
    Toggler slideMotorToggler = new Toggler();


    public DriveController(DcMotorEx frontLeft, DcMotorEx backLeft, DcMotorEx frontRight, DcMotorEx backRight, DcMotorEx slideRotatorLeft, DcMotorEx slideRotatorRight, DcMotorEx slideMotor, DcMotorEx liftMotor) {
        this.frontLeft = frontLeft;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.backRight = backRight;
        this.slideRotatorLeft = slideRotatorLeft;
        this.slideRotatorRight = slideRotatorRight;
        this.slideMotor = slideMotor;
        this.liftMotor = liftMotor;
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
        liftMotorDownPos = liftMotor.getCurrentPosition();

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
//        if(speedFactor + Math.abs(rightX) > 1) {
//            v1 = (r * Math.cos(robotAngle) + rightX) / speedFactor + Math.abs(rightX);
//            v2 = (r * Math.sin(robotAngle) - rightX) / speedFactor + Math.abs(rightX);
//            v3 = (r * Math.sin(robotAngle) + rightX) / speedFactor + Math.abs(rightX);
//            v4 = (r * Math.cos(robotAngle) - rightX) / speedFactor + Math.abs(rightX);
//        } else {
            v1 = r * Math.cos(robotAngle) + rightX;
            v2 = r * Math.sin(robotAngle) - rightX;
            v3 = r * Math.sin(robotAngle) + rightX;
            v4 = r * Math.cos(robotAngle) - rightX;
//        }

        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setPower(v1 * speedFactor);
        frontRight.setPower(v2 * speedFactor);
        backLeft.setPower(v3 * speedFactor);
        backRight.setPower(v4 * speedFactor);
    }

    void setMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        backLeft.setMode(mode);
        frontRight.setMode(mode);
        backRight.setMode(mode);
    }

    public void checkAndToggleRotator(boolean isPressed) {

        if(slideRotatorToggler.toggle(isPressed)) {
            slideRotatorDown = !slideRotatorDown;

            if(slideRotatorDown) {
                slideRotatorRight.setTargetPosition(slideRotatorDownPosRight);
                slideRotatorLeft.setTargetPosition(slideRotatorDownPosLeft);
            } else {
                slideRotatorRight.setTargetPosition(slideRotatorDownPosRight - 280);
                slideRotatorLeft.setTargetPosition(slideRotatorDownPosLeft - 280);
            }

            slideRotatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRotatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slideRotatorLeft.setPower(0.8);
            slideRotatorRight.setPower(0.8);
        }
    }

    public void rotateArm(float power) {

        if (slideMotor.getCurrentPosition() > 1200) {
            slideMotor.setTargetPosition(1200);
        }
        if (Math.abs(power) >= 0.1) {
            slideRotatorToggler.toggle(false);
            if (power > 0) {
                slideRotatorRight.setTargetPosition(slideRotatorDownPosRight-360);
                slideRotatorLeft.setTargetPosition(slideRotatorDownPosLeft-360);
            } else {
                slideRotatorRight.setTargetPosition(slideRotatorDownPosRight);
                slideRotatorLeft.setTargetPosition(slideRotatorDownPosLeft);
            }

            slideRotatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRotatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slideRotatorRight.setPower(Math.abs(power));
            slideRotatorLeft.setPower(Math.abs(power));
        } else {
           if (slideRotatorToggler.toggle(true)) {
               slideRotatorStoppedPosLeft = slideRotatorLeft.getCurrentPosition();
               slideRotatorStoppedPosRight = slideRotatorRight.getCurrentPosition();
               slideRotatorRight.setTargetPosition(slideRotatorRight.getCurrentPosition());
               slideRotatorLeft.setTargetPosition(slideRotatorLeft.getCurrentPosition());

               slideRotatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               slideRotatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

               slideRotatorRight.setPower(0.5f);
               slideRotatorLeft.setPower(0.5f);
           }
        }
    }

    //TODO: make an arm rotation enum and toggler just like claw position toggler

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

    /**
     *
     * @param tiles distance in floor tiles to move.
     *
     * @implNote will only work if the drive controller was initialized with encoder mode as true.
     */

    public void forwards(double tiles, double power) {
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + (int) Math.round(tiles * tilesToPos));
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + (int) Math.round(tiles * tilesToPos));
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + (int) Math.round(tiles * tilesToPos));
        backRight.setTargetPosition(backRight.getCurrentPosition() + (int) Math.round(tiles * tilesToPos));
//        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + (int) Math.round(tiles * tilesToPosFL));
//        frontRight.setTargetPosition(frontRight.getCurrentPosition() + (int) Math.round(tiles * tilesToPosFR));
//        backLeft.setTargetPosition(backLeft.getCurrentPosition() + (int) Math.round(tiles * tilesToPosBL));
//        backRight.setTargetPosition(backRight.getCurrentPosition() + (int) Math.round(tiles * tilesToPosBR));

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForMotors();
    }

    /**
     *
     * @param tiles distance in floor tiles to move.
     *
     */

    public void backwards(double tiles, double power) {
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - (int) Math.round(tiles * tilesToPos));
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - (int) Math.round(tiles * tilesToPos));
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - (int) Math.round(tiles * tilesToPos));
        backRight.setTargetPosition(backRight.getCurrentPosition() - (int) Math.round(tiles * tilesToPos));
//        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - (int) Math.round(tiles * tilesToPosFL));
//        frontRight.setTargetPosition(frontRight.getCurrentPosition() - (int) Math.round(tiles * tilesToPosFR));
//        backLeft.setTargetPosition(backLeft.getCurrentPosition() - (int) Math.round(tiles * tilesToPosBL));
//        backRight.setTargetPosition(backRight.getCurrentPosition() - (int) Math.round(tiles * tilesToPosBR));

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForMotors();
    }

    /**
     *
     * @param tiles distance in floor tiles to move.
     *
     */

    public void left(double tiles, double power) {
        frontLeft.setTargetPosition((int) Math.round(frontLeft.getCurrentPosition() - (int) Math.round(tiles * tilesToPos) * 1.15));
        frontRight.setTargetPosition((int) Math.round(frontRight.getCurrentPosition() + (int) Math.round(tiles * tilesToPos) * 1.15));
        backLeft.setTargetPosition((int) Math.round(backLeft.getCurrentPosition() + (int) Math.round(tiles * tilesToPos) * 1.15));
        backRight.setTargetPosition((int) Math.round(backRight.getCurrentPosition() - (int) Math.round(tiles * tilesToPos) * 1.15));
//        frontLeft.setTargetPosition((int) Math.round(frontLeft.getCurrentPosition() - (int) Math.round(tiles * tilesToPosFL) * 1.15));
//        frontRight.setTargetPosition((int) Math.round(frontRight.getCurrentPosition() + (int) Math.round(tiles * tilesToPosFR) * 1.15));
//        backLeft.setTargetPosition((int) Math.round(backLeft.getCurrentPosition() + (int) Math.round(tiles * tilesToPosBL) * 1.15));
//        backRight.setTargetPosition((int) Math.round(backRight.getCurrentPosition() - (int) Math.round(tiles * tilesToPosBR) * 1.15));
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForMotors();
    }

    /**
     *
     * @param tiles distance in floor tiles to move.
     *
     */

    public void right(double tiles, double power) {
        frontLeft.setTargetPosition((int) Math.round(frontLeft.getCurrentPosition() + (int) Math.round(tiles * tilesToPos) * 1.15));
        frontRight.setTargetPosition((int) Math.round(frontRight.getCurrentPosition() - (int) Math.round(tiles * tilesToPos) * 1.15));
        backLeft.setTargetPosition((int) Math.round(backLeft.getCurrentPosition() - (int) Math.round(tiles * tilesToPos) * 1.15));
        backRight.setTargetPosition((int) Math.round(backRight.getCurrentPosition() + (int) Math.round(tiles * tilesToPos) * 1.15));
//        frontLeft.setTargetPosition((int) Math.round(frontLeft.getCurrentPosition() + (int) Math.round(tiles * tilesToPosFL) * 1.15));
//        frontRight.setTargetPosition((int) Math.round(frontRight.getCurrentPosition() - (int) Math.round(tiles * tilesToPosFR) * 1.15));
//        backLeft.setTargetPosition((int) Math.round(backLeft.getCurrentPosition() - (int) Math.round(tiles * tilesToPosBL) * 1.15));
//        backRight.setTargetPosition((int) Math.round(backRight.getCurrentPosition() + (int) Math.round(tiles * tilesToPosBR) * 1.15));
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForMotors();

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

    public void turnRight(double degrees, double power) {
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + (int) Math.round(degrees * degreesToPos));
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - (int) Math.round(degrees * degreesToPos));
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + (int) Math.round(degrees * degreesToPos));
        backRight.setTargetPosition(backRight.getCurrentPosition() - (int) Math.round(degrees * degreesToPos));
//        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + (int) Math.round(degrees * degreesToPosFL));
//        frontRight.setTargetPosition(frontRight.getCurrentPosition() - (int) Math.round(degrees * degreesToPosFR));
//        backLeft.setTargetPosition(backLeft.getCurrentPosition() + (int) Math.round(degrees * degreesToPosBL));
//        backRight.setTargetPosition(backRight.getCurrentPosition() - (int) Math.round(degrees * degreesToPosBR));
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        waitForMotors();
    }

    public void turnLeft(double degrees, double power) {
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - (int) Math.round(degrees * degreesToPos));
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + (int) Math.round(degrees * degreesToPos));
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - (int) Math.round(degrees * degreesToPos));
        backRight.setTargetPosition(backRight.getCurrentPosition() + (int) Math.round(degrees * degreesToPos));
//        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - (int) Math.round(degrees * degreesToPosFL));
//        frontRight.setTargetPosition(frontRight.getCurrentPosition() + (int) Math.round(degrees * degreesToPosFR));
//        backLeft.setTargetPosition(backLeft.getCurrentPosition() - (int) Math.round(degrees * degreesToPosBL));
//        backRight.setTargetPosition(backRight.getCurrentPosition() + (int) Math.round(degrees * degreesToPosBR));
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(-power);
        backRight.setPower(power);
        waitForMotors();
    }


    public void setArmScoringPos(float power) {
        slideRotatorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRotatorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(slideRotatorRight.getCurrentPosition() > slideRotatorDownPosRight - 380 && slideRotatorLeft.getCurrentPosition() > slideRotatorDownPosLeft - 380) {
            if(slideRotatorRight.getCurrentPosition() < slideRotatorDownPosRight - 300) {
                slideRotatorRight.setPower(-0.2);
                slideRotatorLeft.setPower(-0.2);
            } else {
                slideRotatorRight.setPower(-power);
                slideRotatorLeft.setPower(-power);
            }
        }
        slideRotatorRight.setTargetPosition(slideRotatorDownPosRight - 380);
        slideRotatorLeft.setTargetPosition(slideRotatorDownPosLeft - 380);
        slideRotatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRotatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRotatorRight.setPower(power);
        slideRotatorLeft.setPower(power);
    }

    public void setArmGrabbingPos(float power) {
        slideRotatorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRotatorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(slideRotatorRight.getCurrentPosition() < slideRotatorDownPosRight - 20 && slideRotatorLeft.getCurrentPosition() < slideRotatorDownPosLeft - 20) {
            if(slideRotatorRight.getCurrentPosition() > slideRotatorDownPosRight - 100) {
                slideRotatorRight.setPower(0);
                slideRotatorLeft.setPower(0);
            } else {
                slideRotatorRight.setPower(power);
                slideRotatorLeft.setPower(power);
            }
        }
        slideRotatorRight.setTargetPosition(slideRotatorDownPosRight);
        slideRotatorLeft.setTargetPosition(slideRotatorDownPosLeft);
        slideRotatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRotatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRotatorRight.setPower(power);
        slideRotatorLeft.setPower(power);
    }

    /**
     *
     * @param height between 0.0 and 1.0, bottom and top respectively
     * @param power
     */
    public void setSlidePos(float height, float power) {
        slideMotor.setTargetPosition((int) (slideMotorDownPos + (height * 2800)));
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(power);
    }

    public void setLifterStringLength() {
        float armLengthMM = (slideMotorDownPos - slideMotor.getCurrentPosition()) * slidePosToArmLengthMM + armLengthConstant;
        float armAngle = (slideRotatorDownPosRight - slideRotatorRight.getCurrentPosition()) * ArmRotatorPosToArmAngle;

        double stringLenMM = Math.sqrt(Math.pow(100 + (-armLengthMM*Math.cos(armAngle)), 2) + Math.pow(48 + (armLengthMM * Math.sin(armAngle)), 2));

        double stringLenChange = stringLenMM - currentStringLength;
        int posToGo = (int) (stringLengthToStringLifterPos*stringLenChange);
        currentStringLength = stringLenMM;

        liftMotor.setTargetPosition(liftMotor.getCurrentPosition() + posToGo);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1);
    }

}