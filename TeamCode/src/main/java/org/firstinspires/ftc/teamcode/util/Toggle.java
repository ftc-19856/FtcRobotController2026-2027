package org.firstinspires.ftc.teamcode.util;

/** Rising-edge button toggle. */
public final class Toggle {
    private boolean state;
    private boolean wasPressed;

    public Toggle(boolean initialState) {
        this(initialState, false);
    }

    /** Seeds the button latch so a button held across START is not a fresh press. */
    public Toggle(boolean initialState, boolean initiallyPressed) {
        state = initialState;
        wasPressed = initiallyPressed;
    }

    public void update(boolean pressed) {
        if (pressed && !wasPressed) {
            state = !state;
        }
        wasPressed = pressed;
    }

    public boolean get() {
        return state;
    }
}
