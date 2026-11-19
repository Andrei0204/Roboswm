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
         //distance (in), rpm, hood servo position
                add(27.3, 2338, 0.297); // noua
                add(31.4, 2328, 0.291); // noua
                add(33.7, 2328, 0.285); // noua
                add(34.6, 2403, 0.283); // noua
                add(37.8, 2328, 0.283); // noua
                add(42.2, 2348, 0.275); // noua
                add(43.5, 2358, 0.273); // noua
                add(45.9, 2433, 0.275); // noua
                add(46.5, 2418, 0.267); // noua
                add(50.5, 2428, 0.265); // noua
                add(53.3, 2463, 0.263); // noua
                add(59.4, 2403, 0.283); // noua

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

    public static double minDistance() {
        return TABLE.firstKey();
    }

    public static double maxDistance() {
        return TABLE.lastKey();
    }

    public static double getRpm(double distance) {
        return get(distance).rpm;
    }

    public static double getHood(double distance) {
        return get(distance).hood;
    }
}
