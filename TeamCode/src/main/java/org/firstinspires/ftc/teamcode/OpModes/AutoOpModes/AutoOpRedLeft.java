package org.firstinspires.ftc.teamcode.OpModes.AutoOpModes;

import static com.google.blocks.ftcrobotcontroller.util.CurrentGame.TFOD_LABELS;
import static com.google.blocks.ftcrobotcontroller.util.CurrentGame.TFOD_MODEL_ASSET;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Helpers.CameraController;
import org.firstinspires.ftc.teamcode.Helpers.ClawController;
import org.firstinspires.ftc.teamcode.Helpers.DriveController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous
public class AutoOpRedLeft extends LinearOpMode {

    DcMotorEx frontLeft, frontRight, backLeft, backRight, slideRotatorLeft, slideRotatorRight, slideMotor, liftMotor;
    DriveController driveController;

    Servo leftClaw, rightClaw, clawServo, planeLauncher;
    ClawController clawController;


    CameraController cameraController;


    double driveSpeed = .25;
    double rotateSpeed = .5;

    void initialize() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        slideRotatorLeft = hardwareMap.get(DcMotorEx.class, "slideRotatorLeft");
        slideRotatorRight = hardwareMap.get(DcMotorEx.class, "slideRotatorRight");
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");

        driveController = new DriveController(frontLeft, backLeft, frontRight, backRight, slideRotatorLeft, slideRotatorRight, slideMotor, liftMotor);
        driveController.init();

        planeLauncher = hardwareMap.get(Servo.class, "planeLauncher");

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        clawController = new ClawController(leftClaw, rightClaw, clawServo);
    }



    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        // CAMERA DETECTING
        int loc = -1;

        while(!isStarted()) {
            // sense location

            loc = cameraController.detectProp();

            telemetry.addLine(String.valueOf(loc));
            telemetry.update();
        }


        telemetry.addData("Location:", loc);
        telemetry.update();

        //CAMERA DETECTION PROCESSING



        //loc is where the model found the team prop
        driveController.forwards(1/8f,driveSpeed); //robot center on tile center - TO BE ADJUSTED
        driveController.right(3/32f,driveSpeed);
        driveController.turnRight(180,rotateSpeed); //(mech arm forward)
        clawController.setClawLevelPos();
        sleep(250);
        driveController.backwards(4/4f,driveSpeed); //robot center on tile border center
        //adjust how close the bot needs to be depending on arm length

        clawController.setClawLevelPos();
        sleep(250);

        if (loc == 0) {
            driveController.turnLeft(90f,rotateSpeed);
            driveController.left(1/4f, driveSpeed);
            driveController.backwards(1/16f,driveSpeed);
            sleep(250);//place purple pixel on left tape - left claw
            clawController.toggleLeftClaw();
            sleep(250);
            clawController.setClawScoringPos();
            driveController.right(1/4f, driveSpeed);
            driveController.forwards(1/16f,driveSpeed);
            driveController.turnRight(90f,rotateSpeed);
        }
        if (loc == 1) {
            driveController.backwards(1/8f,driveSpeed);
            sleep(250);//place purple pixel on mid tape - left claw
            clawController.toggleLeftClaw();
            sleep(250);
            clawController.setClawScoringPos();
            driveController.forwards(1/8f,driveSpeed);
        }
        if (loc == 2) {
            driveController.turnRight(90f,rotateSpeed);
            driveController.right(1/4f, driveSpeed);
            driveController.backwards(1/16f,driveSpeed);
            sleep(250);//place purple pixel on right tape - left claw
            clawController.toggleLeftClaw();
            sleep(250);
            clawController.setClawScoringPos();
            driveController.left(1/4, driveSpeed);
            driveController.forwards(1/16f,driveSpeed);
            driveController.turnLeft(90f,rotateSpeed);
        }
        driveController.forwards(4/4f,driveSpeed);
        //ends on the border of the two tiles -ideally


        driveController.turnLeft(90,rotateSpeed);


        driveController.forwards(17/16f,driveSpeed); //moves to center of tile 3
        driveController.left(15/8f,driveSpeed); //center of left door tile
        driveController.forwards(5/2f,driveSpeed); //border between two tiles left of backstage
        driveController.right(5/4f,driveSpeed); //aligned with center of backstage

        //rotate arm and toggle claw pos
        clawController.setClawLevelPos();
        sleep(500);
        driveController.setArmScoringPos(0.5f);
        driveController.setSlidePos(0.3f, 1f);
        sleep(1000);
        clawController.setClawScoringPos();
        sleep(500);

        //this is kinda not made up anymore, but def tweak the numbers yknow...
        if (loc == 2) { //right
            driveController.right(3/16f,driveSpeed);
            sleep(500);
            clawController.toggleRightClaw();
            sleep(500);
            driveController.left(3/8f,driveSpeed);
            driveController.backwards(1/4f,driveSpeed);
        }
        if (loc == 1) { //mid
            sleep(500);
            clawController.toggleRightClaw();
            driveController.backwards(1/4f,driveSpeed);
            driveController.left(3/16f,driveSpeed);
        }
        if (loc == 0) { //left
            driveController.left(3/16f,driveSpeed); //TODO: double check
            sleep(500);
            clawController.toggleRightClaw();
            sleep(500);
            driveController.backwards(1/4f,driveSpeed);
            driveController.right(1/2f,driveSpeed);
        }
        driveController.setSlidePos(0, 0.4f);
        clawController.setClawLevelPos();
        driveController.setArmGrabbingPos(0.4f);
        sleep(500);

        driveController.left(1/4f,driveSpeed); //TODO: double check - park
        driveController.forwards(9/8f,driveSpeed);


    }


}
