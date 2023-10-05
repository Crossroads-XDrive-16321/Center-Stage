package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Helpers.HelperFunctions;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;


import java.util.ArrayList;
import java.util.List;

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

        Mat leftCrop;
        Mat midCrop;
        Mat rightCrop;
        Mat output = new Mat();
        Scalar leftCol = new Scalar(255.0, 0.0, 0.0);
        Scalar midCol = new Scalar(0.0, 255.0, 0.0);
        Scalar rightCol = new Scalar(0.0, 0.0, 255.0);

//                                              target pink (red)               target sky (blue)
        Scalar[] targetCols = new Scalar[]{new Scalar(255.0, 0.0, 0.0), new Scalar(0.0, 0.0, 255.0)};

        Scalar[] colDiff = new Scalar[]{new Scalar(0.0, 0.0, 0.0), new Scalar(0.0, 0.0, 0.0)};


        public Mat processFrame(Mat input) {

            telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(1, 1, 219, 359);
            Rect midRect = new Rect(220, 1, 199, 359);
            Rect rightRect = new Rect(420, 1, 219, 359);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, leftCol, 1);
            Imgproc.rectangle(output, midRect, midCol, 1);
            Imgproc.rectangle(output, rightRect, rightCol, 1);

            leftCrop = input.submat(leftRect);
            midCrop = input.submat(midRect);
            rightCrop = input.submat(rightRect);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar midavg = Core.mean(midCrop);
            Scalar rightavg = Core.mean(rightCrop);

//            idea for easy threshold: compare the average color values with target color values and calculate a percentage
//            similarity, then if that percentage is above a threshold boom


            for (int i = 0; i < targetCols.length; i++){
                Double leftAvg = HelperFunctions.compareScalars(leftavg, targetCols[i])/3;
                Double midAvg = HelperFunctions.compareScalars(midavg, targetCols[i])/3;
                Double rightAvg = HelperFunctions.compareScalars(rightavg, targetCols[i])/3;

                telemetry.addData("Left sim to"+i, leftAvg);
                telemetry.addData("Mid sim to"+i, midAvg);
                telemetry.addData("Right sim to"+i, rightAvg);

            }


//            for (int i = 0; i<=2; i++) {
//                leftavg.val[i] = Math.floor(leftavg.val[i]);
//                midavg.val[i] = Math.floor(midavg.val[i]);
//                rightavg.val[i] = Math.floor(rightavg.val[i]);
//            }

//            telemetry.addData("Left Avg:", leftavg);
//            telemetry.addData("Mid Avg:", midavg);
//            telemetry.addData("Right Avg:", rightavg);

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
