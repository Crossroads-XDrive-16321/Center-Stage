package org.firstinspires.ftc.teamcode.Helpers;

import com.qualcomm.robotcore.hardware.Servo;

// left open = 0.85, closed = 0.5
// right open = 0.25, closed = 0.6

public class ClawController {

    final double leftOpenPos = 0.88;
    final double leftClosedPos = 0.53;
    final double rightOpenPos = 0.53;
    final double rightClosedPos = 0.88;

    public enum ClawPosition {
        LEVEL,
        SCORING
    }

    public ClawPosition currentClawPos = ClawPosition.SCORING;
    private final float clawLevelPos = 0.63f;
    private final float clawScoringPos = 0f;

    Servo leftServo, rightServo, clawServo;
    public boolean leftIsOpen = false;
    public boolean rightIsOpen = false;
    Toggler clawArmToggler = new Toggler();
    Toggler rightClawPosToggler = new Toggler();
    Toggler leftClawPosToggler = new Toggler();

    public ClawController(Servo leftServo, Servo rightServo, Servo clawServo) {
        this.leftServo = leftServo;
        this.rightServo = rightServo;
        this.clawServo = clawServo;
    }


    public void setClawScoringPos() {
        clawServo.setPosition(clawScoringPos);
    }

    public void setClawLevelPos() {
        clawServo.setPosition(clawLevelPos);
    }

    public void toggleClawPosition(boolean button_pressed) {
        if(clawArmToggler.toggle(button_pressed)) {
            switch (currentClawPos) {
                case LEVEL:
                    clawServo.setPosition(clawScoringPos);
                    currentClawPos = ClawPosition.SCORING;
                    break;
                case SCORING:
                    clawServo.setPosition(clawLevelPos);
                    currentClawPos = ClawPosition.LEVEL;
                    break;
            }
        }
    }

    public boolean toggleRightClaw() {
        if(rightIsOpen) {
            rightServo.setPosition(rightClosedPos);
        } else {
            rightServo.setPosition(rightOpenPos);
        }

        rightIsOpen = !rightIsOpen;

        return(rightIsOpen);
    }

    public boolean toggleLeftClaw() {
        if(leftIsOpen) {
            leftServo.setPosition(leftClosedPos);
        } else {
            leftServo.setPosition(leftOpenPos);
        }

        leftIsOpen = !leftIsOpen;

        return(leftIsOpen);
    }
    public void checkAndToggle(boolean leftButton, boolean rightButton) {

        if(rightClawPosToggler.toggle(rightButton)) {
            toggleRightClaw();
        }

        if(leftClawPosToggler.toggle(leftButton)) {
            toggleLeftClaw();
        }

    }

    public void checkAndToggleHold(boolean leftButton, boolean rightButton) {

        if(leftButton) {
            leftServo.setPosition(leftOpenPos);
        } else {
            leftServo.setPosition(leftClosedPos);
        }

        if(rightButton) {
            rightServo.setPosition(rightOpenPos);
        } else {
            rightServo.setPosition(rightClosedPos);
        }

    }

}
