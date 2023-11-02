package org.firstinspires.ftc.teamcode.Helpers;

import com.qualcomm.robotcore.hardware.Servo;

// left open = 0.85, closed = 0.5
// right open = 0.25, closed = 0.6

public class ClawController {

    final double leftOpenPos = 0.85;
    final double leftClosedPos = 0.5;
    final double rightOpenPos = 0.25;
    final double rightClosedPos = 0.6;

    public enum ClawPosition {
        LEVEL,
        SCORING
    }

    public ClawPosition currentPos = ClawPosition.LEVEL;
    private final float clawLevelPos = 0.13f;
    private final float clawScoringPos = 0.78f;

    Servo leftServo, rightServo, clawServo;
    public boolean leftIsOpen = true;
    public boolean rightIsOpen = true;
    Toggler clawArmToggler = new Toggler();
    Toggler rightClawPosToggler = new Toggler();
    Toggler leftClawPosToggler = new Toggler();

    public ClawController(Servo leftServo, Servo rightServo, Servo clawServo) {
        this.leftServo = leftServo;
        this.rightServo = rightServo;
        this.clawServo = clawServo;
    }


    public void toggleClawPosition(boolean button_pressed) {
        if(clawArmToggler.toggle(button_pressed)) {
            switch (currentPos) {
                case LEVEL:
                    clawServo.setPosition(clawScoringPos);
                    currentPos = ClawPosition.SCORING;
                    break;
                case SCORING:
                    clawServo.setPosition(clawLevelPos);
                    currentPos = ClawPosition.LEVEL;
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

}
