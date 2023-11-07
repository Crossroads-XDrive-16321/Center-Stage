package org.firstinspires.ftc.teamcode.Helpers;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class IMUController {

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    Telemetry telemetry;

    public IMUController(BNO055IMU imu, Telemetry telemetry) {
        this.imu = imu;
        this.telemetry = telemetry;
    }

    public void init() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

        correction = checkDirection();

        // make sure the imu gyro is calibrated before continuing.
        while (!imu.isGyroCalibrated())
        {
            try {
                Thread.sleep(50);
            } catch(InterruptedException e) {
                throw new RuntimeException("Uncaught", e);
            }
        }

    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = lastAngles.firstAngle - angles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = 1;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, - is left + is right
     */
    public void rotate(int degrees, double power, DriveController driveController)
    {
        double leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        telemetry.addLine("Angle Reset");

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn left.
            telemetry.addLine("Turning Left");
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn right.
            telemetry.addLine("Turning Right");
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        driveController.turn(leftPower, rightPower);

        telemetry.addLine("Wheels Set");

        // rotate until turn is completed.
        if (degrees < 0) {

            while (getAngle() == 0) {}

            while (getAngle() > degrees) {
                if (getAngle() - degrees < 15) {
                    leftPower = -0.1;
                    rightPower = 0.1;
                    driveController.turn(leftPower, rightPower);
                }
            }

        } else {    // right turn.
            while (getAngle() == 0) {}

            while (getAngle() < degrees) {
                if(degrees - getAngle() < 15) {
                    leftPower = 0.1;
                    rightPower = -0.1;
                    driveController.turn(leftPower, rightPower);
                }
            }
        }

        // turn the motors off.
        driveController.turn(0, 0);

        // wait for rotation to stop.
        try {
            Thread.sleep(250);
        } catch(InterruptedException e) {
            throw new RuntimeException("Uncaught", e);
        }

        // reset angle tracking on new heading.
        resetAngle();
    }

}
