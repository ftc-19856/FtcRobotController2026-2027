package Components;

/**
 * A clean and reliable toggle handler for robotics.
 *
 * Usage example:
 *
 * Components.Toggle intakeToggle = new Components.Toggle();
 *
 * // Inside your loop:
 * intakeToggle.update(gamepad1.a);
 * if (intakeToggle.getState()) {
 *     // Turn intake on
 * } else {
 *     // Turn intake off
 * }
 */
public class Toggle {

    private boolean currentState;   // The current ON/OFF toggle value
    private boolean lastButtonState = false; // The previous raw button input


    public Toggle(boolean initialState) {
        this.currentState = initialState;
    }

    /**
     * Updates the toggle based on the current button input.
     * This should be called once per loop.
     *
     * @param buttonPressed - the current state of the button (true if pressed)
     */
    public void update(boolean buttonPressed) {
        // Detect the "rising edge" — button goes from unpressed → pressed
        if (buttonPressed && !lastButtonState) {
            currentState = !currentState; // Flip toggle
        }
        lastButtonState = buttonPressed; // Store for next update
    }

    /**
     * Returns the current state of the toggle (true = ON, false = OFF)
     */
    public boolean getState() {
        return currentState;
    }

    /**
     * Manually reset or set the toggle.
     */
    public void setState(boolean newState) {
        currentState = newState;
    }

    /**
     * Returns true if the button was *just pressed* this cycle.
     * (useful if you also want edge detection)
     */
    public boolean wasJustPressed(boolean buttonPressed) {
        return buttonPressed && !lastButtonState;
    }
}
