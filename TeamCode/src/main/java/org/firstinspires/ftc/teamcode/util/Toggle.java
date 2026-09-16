package org.firstinspires.ftc.teamcode.util;

/** Rising-edge button toggle. */
public final class Toggle {
    private boolean state;
    private boolean wasPressed;

    public Toggle(boolean initialState) {
        state = initialState;
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
