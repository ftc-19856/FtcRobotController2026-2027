package org.firstinspires.ftc.teamcode.opmode;

/** Counts a stable yellow -> not-yellow transition, only during collection. */
final class PollenPassCounter {
    static final double DEBOUNCE_SECONDS = 0.04;
    private int count;
    private boolean armed;
    private Boolean lastYellow;
    private double changedAt;

    void reset() {
        count = 0;
        armed = false;
        lastYellow = null;
    }

    void update(boolean yellow, boolean collecting, double seconds) {
        if (!collecting) {
            armed = false;
            lastYellow = null;
            return;
        }
        if (lastYellow == null || lastYellow != yellow) {
            lastYellow = yellow;
            changedAt = seconds;
        }
        if (seconds - changedAt < DEBOUNCE_SECONDS) return;
        if (yellow) {
            armed = true;
        } else if (armed) {
            count = Math.min(4, count + 1);
            armed = false;
        }
    }

    int count() { return count; }
}
