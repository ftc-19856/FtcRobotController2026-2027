package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.RobotConfig;

/** Parking around the reported one-inch hive wall at X=95, Y=50..90. */
public final class BlueHiveParking {
    private static final double WALL_LEFT = 94.5, WALL_RIGHT = 95.5;
    private static final double WALL_BOTTOM = 50, WALL_TOP = 90;
    private static final double CLEARANCE = Math.hypot(RobotConfig.ROBOT_LENGTH_INCHES / 2,
            RobotConfig.ROBOT_WIDTH_INCHES / 2) + 2;
    static final double LEFT = WALL_LEFT - CLEARANCE, RIGHT = WALL_RIGHT + CLEARANCE;
    static final double BOTTOM = WALL_BOTTOM - CLEARANCE, TOP = WALL_TOP + CLEARANCE;
    private static final double EPS = 1e-7;

    private BlueHiveParking() { }

    public static Path route(Pose current, Pose park, Pose control) {
        if (!outside(current)) {
            // Leave the turning buffer without rotating, if the current footprint
            // has at least an inch of physical clearance from the wall.
            Pose escape = escapePose(current);
            if (escape == null) return null;
            Path remainder = route(escape, park, control);
            return remainder == null ? null : path(
                    line(current, escape).constant(current.heading()), remainder);
        }
        if (!outside(park)) return null;
        if (clearCurve(current, control, park, 0)) {
            return curve(current, control, park).linear(current, park);
        }
        // The park and supplied control point are east of the structure.
        List<Pose> top = around(current, TOP);
        List<Pose> bottom = around(current, BOTTOM);
        double topCost = cost(top, park, control), bottomCost = cost(bottom, park, control);
        if (!Double.isFinite(Math.min(topCost, bottomCost))) return null;
        List<Pose> chosen = topCost <= bottomCost ? top : bottom;
        List<Path> legs = new ArrayList<>();
        for (int i = 1; i < chosen.size(); i++) {
            if (distance(chosen.get(i - 1), chosen.get(i)) > EPS) {
                legs.add(line(chosen.get(i - 1), chosen.get(i)));
            }
        }
        legs.add(curve(chosen.get(chosen.size() - 1), control, park));
        return path(legs.toArray(new Path[0])).linear(current, park);
    }

    private static List<Pose> around(Pose start, double y) {
        Pose east = new Pose(RIGHT, y, start.heading());
        List<Pose> points = new ArrayList<>();
        points.add(start);
        if (!clearSegment(start, east)) {
            Pose west = new Pose(LEFT, y, start.heading());
            if (!clearSegment(start, west)) return null;
            points.add(west);
        }
        points.add(east);
        return points;
    }

    private static double cost(List<Pose> points, Pose park, Pose control) {
        if (points == null) return Double.POSITIVE_INFINITY;
        Pose last = points.get(points.size() - 1);
        if (!clearCurve(last, control, park, 0)) return Double.POSITIVE_INFINITY;
        double length = 0;
        for (int i = 1; i < points.size(); i++) length += distance(points.get(i - 1), points.get(i));
        Path tail = curve(last, control, park).constant(park);
        Pose previous = last;
        for (int i = 1; i <= 40; i++) {
            Pose next = tail.get(i / 40.0);
            length += distance(previous, next);
            previous = next;
        }
        return length;
    }

    private static Pose escapePose(Pose p) {
        double c = Math.abs(Math.cos(p.heading())), s = Math.abs(Math.sin(p.heading()));
        double halfX = c * RobotConfig.ROBOT_LENGTH_INCHES / 2 + s * RobotConfig.ROBOT_WIDTH_INCHES / 2;
        double halfY = s * RobotConfig.ROBOT_LENGTH_INCHES / 2 + c * RobotConfig.ROBOT_WIDTH_INCHES / 2;
        if (p.x() + halfX <= WALL_LEFT - 1) return new Pose(LEFT, p.y(), p.heading());
        if (p.x() - halfX >= WALL_RIGHT + 1) return new Pose(RIGHT, p.y(), p.heading());
        if (p.y() - halfY >= WALL_TOP + 1) return new Pose(p.x(), TOP, p.heading());
        if (p.y() + halfY <= WALL_BOTTOM - 1) return new Pose(p.x(), BOTTOM, p.heading());
        return null;
    }

    static boolean outside(Pose p) {
        return p.x() <= LEFT + EPS || p.x() >= RIGHT - EPS
                || p.y() <= BOTTOM + EPS || p.y() >= TOP - EPS;
    }

    static boolean clearSegment(Pose a, Pose b) {
        double enter = 0, exit = 1;
        double[] origin = {a.x(), a.y()}, delta = {b.x() - a.x(), b.y() - a.y()};
        double[] low = {LEFT + EPS, BOTTOM + EPS}, high = {RIGHT - EPS, TOP - EPS};
        for (int axis = 0; axis < 2; axis++) {
            if (Math.abs(delta[axis]) < EPS) {
                if (origin[axis] <= low[axis] || origin[axis] >= high[axis]) return true;
            } else {
                double t1 = (low[axis] - origin[axis]) / delta[axis];
                double t2 = (high[axis] - origin[axis]) / delta[axis];
                enter = Math.max(enter, Math.min(t1, t2));
                exit = Math.min(exit, Math.max(t1, t2));
            }
        }
        return exit <= enter;
    }

    private static boolean clearCurve(Pose a, Pose control, Pose b, int depth) {
        // A Bezier lies inside its control hull. Recursively split intersecting
        // hulls; an unresolved intersection is rejected conservatively.
        if (Math.max(a.x(), Math.max(control.x(), b.x())) <= LEFT + EPS
                || Math.min(a.x(), Math.min(control.x(), b.x())) >= RIGHT - EPS
                || Math.max(a.y(), Math.max(control.y(), b.y())) <= BOTTOM + EPS
                || Math.min(a.y(), Math.min(control.y(), b.y())) >= TOP - EPS) return true;
        if (!outside(a) || !outside(b) || depth >= 12) return false;
        Pose ac = midpoint(a, control), cb = midpoint(control, b), middle = midpoint(ac, cb);
        return clearCurve(a, ac, middle, depth + 1) && clearCurve(middle, cb, b, depth + 1);
    }

    private static Pose midpoint(Pose a, Pose b) {
        return new Pose((a.x() + b.x()) / 2, (a.y() + b.y()) / 2);
    }

    private static double distance(Pose a, Pose b) {
        return Math.hypot(a.x() - b.x(), a.y() - b.y());
    }
}
