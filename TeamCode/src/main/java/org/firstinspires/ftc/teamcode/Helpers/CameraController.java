package org.firstinspires.ftc.teamcode.Helpers;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.OpModes.OpenCVExample;
import org.firstinspires.ftc.teamcode.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

public class CameraController {

    TfodProcessor tfod;
    VisionPortal visionPortal;
    OpenCvCamera cam;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    int blue_left = 1;
    int blue_mid = 2;
    int blue_right = 3;
    int red_left = 4;
    int red_mid = 5;
    int red_right = 6;

    public void initAprilTags(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(0.166, 578.272, 578.272, 402.145, 221.506);

        cam.setPipeline(aprilTagDetectionPipeline); //lmao - finding out that this code was missing broke and breaking the whole controller took me 2 hours :D
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                cam.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    public void initTfod(HardwareMap hardwareMap, float minConfidence) {

        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName("model_20240107_104336.tflite")

                .setModelLabels(new String[]{"blue_prop", "red_prop"})
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(false)
                .setModelInputSize(300)
                .setModelAspectRatio(16/9f)

                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 360));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true); // TODO: probably disable this after testing's done

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(minConfidence);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, true);

    }

    public int detectProp() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        float maxConfidence = 0;
        double x = 0;
        for (Recognition recognition : currentRecognitions) {
            if (recognition.getConfidence() > maxConfidence) {
                x = (recognition.getLeft() + recognition.getRight()) / 2;
                maxConfidence = recognition.getConfidence();
            }
        }

        if (x < 640 / 3f && x >= 0) {
            return(0);
        } else if (x < 2 * 640 / 3f && x >= 640 / 3f) {
            return(1);
        } else {
            return(2);
        }
    }

    public AprilTagDetection detectAprilTag(int tagID) {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if(currentDetections.size() != 0) {
            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == tagID) {
                    return (tag);
                }
            }
        }

        return(null);
    }

    public AprilTagDetection detectAprilTag() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        AprilTagDetection returnTag;
        double best = 10;

        for(AprilTagDetection tag : currentDetections) {
            if(Math.abs(tag.pose.x + 0.85) < best) {
                best = tag.pose.x;
                returnTag = tag;
            }
        }

        return(null);
    }

}
