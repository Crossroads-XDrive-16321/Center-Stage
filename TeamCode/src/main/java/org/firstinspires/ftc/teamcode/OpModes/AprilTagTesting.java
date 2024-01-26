package org.firstinspires.ftc.teamcode.OpModes;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;
@TeleOp
public class AprilTagTesting extends LinearOpMode{

    public void runOpMode() {

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setLensIntrinsics(952.817f, 952.817f, 659.539f, 383.43f) //if it doesnt work here: https://www.youtube.com/watch?v=bTcCY3DZM0k
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 360))
                .build();

        waitForStart();

        while (!isStopRequested()) {
            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                telemetry.addData("x",tag.ftcPose.x);
                telemetry.addData("y",tag.ftcPose.y);
                telemetry.addData("z",tag.ftcPose.z);
                telemetry.addData("pitch",tag.ftcPose.pitch);
                telemetry.addData("yaw",tag.ftcPose.yaw);
                telemetry.addData("roll",tag.ftcPose.roll);
                telemetry.addData("tag id", tag.id);

            }

            telemetry.update();
        }
    }
}
