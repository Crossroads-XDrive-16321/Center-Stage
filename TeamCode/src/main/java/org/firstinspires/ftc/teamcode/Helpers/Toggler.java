package org.firstinspires.ftc.teamcode.Helpers;

public class Toggler {

    boolean firstPressed;

    public boolean toggle(boolean isPressed) {
        if(isPressed) {
            if(firstPressed) {
                firstPressed = false;
                return(true);
            } else {
                return(false);
            }
        } else {
            firstPressed = true;
            return(false);
        }
    }

}
