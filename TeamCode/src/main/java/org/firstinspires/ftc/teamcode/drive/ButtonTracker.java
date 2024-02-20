package org.firstinspires.ftc.teamcode.drive;

public class ButtonTracker {

    private boolean buttonState;
    public boolean wasPressed = false;
    public boolean wasReleased = false;

    public void update(boolean newButtonState) {
        if (newButtonState != buttonState) {
            if (buttonState) {
                wasPressed = true;
                wasReleased = false;
            } else {
                wasPressed = false;
                wasReleased = true;
            }
        } else {
            wasPressed = false;
            wasReleased = false;
        }
        buttonState = newButtonState;
    }

}
