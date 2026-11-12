package org.firstinspires.ftc.teamcode.util;

import java.util.Map;
import java.util.TreeMap;

/**
 * Distance -> (flywheel RPM, hood position) lookup with linear interpolation.
 *
 * Distances outside the table are clamped to the nearest end point (no extrapolation),
 * so a bad pose never commands a crazy RPM.
 *
 * Usage:
 *   ShotTable.Shot s = ShotTable.get(distanceInches);
 *   launcher.setTargetRPM(s.rpm);
 *   pivot.setPos(s.hood);
 */
public final class ShotTable {
    private ShotTable() {
    }

    public static final class Shot {
        public final double rpm;
        public final double hood;

        public Shot(double rpm, double hood) {
            this.rpm = rpm;
            this.hood = hood;
        }

        @Override
        public String toString() {
            return "Shot{rpm=" + rpm + ", hood=" + hood + "}";
        }
    }

    private static final TreeMap<Double, Shot> TABLE = new TreeMap<>();

    static {
        // distance (in), rpm, hood servo position
        add(21.7, 2150, 0.316);
        add(26.7, 2202, 0.29);
        add(28.3, 2150, 0.3);
        add(32.1, 2161, 0.295);
        add(35.7, 2190, 0.29);
        add(37.6, 2220, 0.28);
        add(40.6, 2248, 0.276);
        add(42.0, 2200, 0.294);
        add(44.7, 2310, 0.27);
        add(44.9, 2300, 0.272);
        add(47.0, 2300, 0.28);
        add(48.5, 2280, 0.272);
        add(49.4, 2293, 0.271);
    }

    /** Adds or replaces a data point. Call from anywhere to tune the table. */
    public static void add(double distance, double rpm, double hood) {
        TABLE.put(distance, new Shot(rpm, hood));
    }

    /** Interpolated shot for a distance (same units as the table, inches). */
    public static Shot get(double distance) {
        if (TABLE.isEmpty()) throw new IllegalStateException("ShotTable is empty");
        if (Double.isNaN(distance)) return TABLE.firstEntry().getValue();

        Map.Entry<Double, Shot> lo = TABLE.floorEntry(distance);
        Map.Entry<Double, Shot> hi = TABLE.ceilingEntry(distance);

        if (lo == null) return hi.getValue();   // closer than the first point
        if (hi == null) return lo.getValue();   // farther than the last point
        if (lo.getKey().equals(hi.getKey())) return lo.getValue();

        double t = (distance - lo.getKey()) / (hi.getKey() - lo.getKey());
        return new Shot(
                lo.getValue().rpm + t * (hi.getValue().rpm - lo.getValue().rpm),
                lo.getValue().hood + t * (hi.getValue().hood - lo.getValue().hood));
    }

    public static double getRpm(double distance) {
        return get(distance).rpm;
    }

    public static double getHood(double distance) {
        return get(distance).hood;
    }
}
