package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
public class DEBUGOpenCV extends OpMode {

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
        Mat centerCrop;
        double centerAvgR, centerAvgG, centerAvgB;
        Mat centerCropR, centerCropG, centerCropB;
        Mat output = new Mat();
        Scalar centerCol = new Scalar(255.0,0.0,0.0);


        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            Rect centerRect = new Rect(220, 120, 199, 119);

            input.copyTo(output);
            Imgproc.rectangle(output, centerRect, centerCol, 1);

            centerCrop = YCbCr.submat(centerRect);

            //color value is 2 for red
            Core.extractChannel(centerCrop, centerCropR, 0);
            Core.extractChannel(centerCrop, centerCropG, 1);
            Core.extractChannel(centerCrop, centerCropB, 2);

            Scalar centeravgR = Core.mean(centerCropR);
            Scalar centeravgG = Core.mean(centerCropG);
            Scalar centeravgB = Core.mean(centerCropB);

            //make sure the '2' matches with the thing
            centerAvgR = centeravgR.val[0];
            centerAvgG = centeravgG.val[1];
            centerAvgB = centeravgB.val[2];

            telemetry.addData("Red Avg:", centerAvgR);
            telemetry.addData("Green Avg:", centerAvgG);
            telemetry.addData("Blue Avg:", centerAvgB);
            return(output);
            //idea: this code works fine but were just trying to detect a color against a white background, which is 255
            //in all three colors, so with just the red channel for eg. theres no difference between (255,0,0) and (255,255,255)
            //maybe instead of finding the highest target color value, we should find the lowest values of all other colors
            //and so cyan = blue+green so detect for least red and pink = red+1/2green+1/2blue so detect least green+blue

            //idea2: throw the whole ycrcb or whatever out the trash and use only rgb - google says the only reason why
            // ycbcr exists is for more efficient data transfer, which im sure isnt a major concern for robotics when
            // compared to getting the code to work at all

        }

    }

}



//640x360
