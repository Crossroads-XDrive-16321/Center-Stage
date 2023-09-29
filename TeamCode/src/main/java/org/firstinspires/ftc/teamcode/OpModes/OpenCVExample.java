package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous
public class OpenCVExample extends OpMode {

    OpenCvCamera cam = null;

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        cam.setPipeline(new examplePipeline());

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                cam.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    @Override
    public void loop() {
        telemetry.update();
    }

    class examplePipeline extends OpenCvPipeline {

        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat midCrop;
        Mat rightCrop;
        double leftAvg;
        double midAvg;
        double rightAvg;
        Mat output = new Mat();
        Scalar leftCol = new Scalar(255.0, 0.0, 0.0);
        Scalar midCol = new Scalar(0.0, 255.0, 0.0);
        Scalar rightCol = new Scalar(0.0, 0.0, 255.0);


        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(1, 1, 219, 359);
            Rect midRect = new Rect(220, 1, 199, 359);
            Rect rightRect = new Rect(420, 1, 219, 359);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, leftCol, 1);
            Imgproc.rectangle(output, midRect, midCol, 1);
            Imgproc.rectangle(output, rightRect, rightCol, 1);

            leftCrop = YCbCr.submat(leftRect);
            midCrop = YCbCr.submat(midRect);
            rightCrop = YCbCr.submat(rightRect);

            //color value is 2 for red
            Core.extractChannel(leftCrop, leftCrop, 0);
            Core.extractChannel(midCrop, midCrop, 0);
            Core.extractChannel(rightCrop, rightCrop, 0);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar midavg = Core.mean(midCrop);
            Scalar rightavg = Core.mean(rightCrop);

            //make sure the '2' matches with the thing
            leftAvg = leftavg.val[0];
            midAvg = midavg.val[0];
            rightAvg = rightavg.val[0];

            if(leftAvg > midAvg && leftAvg > rightAvg) {
                telemetry.addLine("WOOOOO LEFT");
            } else if(midAvg > leftAvg && midAvg > rightAvg) {
                telemetry.addLine("WOOOOO MIDDLEEE");
            } else {
                telemetry.addLine("WOOOOO DERECHA");
            }

            telemetry.addData("Left Avg:", leftAvg);
            telemetry.addData("Mid Avg:", midAvg);
            telemetry.addData("Right Avg:", rightAvg);
            return(output);

        }

    }

}



//640x360
