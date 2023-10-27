package org.firstinspires.ftc.teamcode.Helpers;

import com.qualcomm.robotcore.hardware.Servo;

// left open = 0.85, closed = 0.5
// right open = 0.25, closed = 0.6

public class ClawController {

    public enum ClawPosition {
        LEVEL,
        SCORING
    }

    public ClawPosition currentPos = ClawPosition.LEVEL;
    private final float clawLevelPos = 0.13f;
    private final float clawScoringPos = 0.78f;

    Servo leftServo, rightServo, clawServo;
    public boolean isOpen = true;
    Toggler clawArmToggler = new Toggler();
    Toggler clawPosToggler = new Toggler();

    public ClawController(Servo leftServo, Servo rightServo, Servo clawServo) {
        this.leftServo = leftServo;
        this.rightServo = rightServo;
        this.clawServo = clawServo;
    }

    public void moveClaw(float dir) { // dir between -1 and 1
        if(dir == 0) {
            clawServo.setPosition(clawServo.getPosition());
        }
        clawServo.setPosition(clawServo.getPosition() + dir/100.0f);
    }

    public void toggleClawPosition(boolean button_pressed) {
        if(clawPosToggler.toggle(button_pressed)) {
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
    void openClaw() {
        leftServo.setPosition(0.85);
        rightServo.setPosition(0.25);
    }

    void closeClaw() {
        leftServo.setPosition(0.5);
        rightServo.setPosition(0.6);
    }

    public boolean toggleClaw() {
        if(isOpen) {
            closeClaw();
        } else {
            openClaw();
        }

        isOpen = !isOpen;

        return(isOpen);
    }
    public void checkAndToggle(boolean isButtonPressed) {

        if(clawArmToggler.toggle(isButtonPressed)) {
            toggleClaw();
        }

    }

}
