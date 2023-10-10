package org.firstinspires.ftc.teamcode.Helpers;

import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.C;

// left open = 0.85, closed = 0.5
// right open = 0.25, closed = 0.6

public class ClawController {

    Servo leftServo, rightServo, clawServo;
    public boolean isOpen = true;
    Toggler toggler = new Toggler();

    public ClawController(Servo leftServo, Servo rightServo, Servo clawServo) {
        this.leftServo = leftServo;
        this.rightServo = rightServo;
        this.clawServo = clawServo;
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

        if(toggler.toggle(isButtonPressed)) {
            toggleClaw();
        }

    }

}
