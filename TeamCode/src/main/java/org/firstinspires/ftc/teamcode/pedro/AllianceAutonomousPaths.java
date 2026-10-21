package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** User-supplied Visualizer paths, using Pedro field inches and Pedro degrees. */
public abstract class AllianceAutonomousPaths {
    public final Pose start;
    public final Pose shooting;
    public final Pose park;
    private final Pose parkControl;

    protected AllianceAutonomousPaths(Pose start, Pose shooting, Pose parkControl, Pose park) {
        this.start = start;
        this.shooting = shooting;
        this.parkControl = parkControl;
        this.park = park;
    }

    public Path path1() {
        return line(start, shooting).linear(start, shooting);
    }

    public Path path2() {
        return curve(shooting, parkControl, park).constant(park);
    }
}
