package org.firstinspires.ftc.teamcode.aiming;

import java.util.Arrays;

/**
 * Piecewise-linear lookup table y(x). Inputs outside the table are clamped to the end values
 * (callers check {@link #minX()} / {@link #maxX()} to flag out-of-range shots).
 */
public final class InterpolatingTable {
    private final double[] xs;
    private final double[] ys;

    /**
     * @param xs strictly increasing sample positions (at least 2)
     * @param ys sample values, same length as xs
     */
    public InterpolatingTable(double[] xs, double[] ys) {
        if (xs == null || ys == null || xs.length != ys.length || xs.length < 2) {
            throw new IllegalArgumentException("InterpolatingTable needs two equal-length arrays with at least 2 points");
        }
        for (int i = 0; i < xs.length; i++) {
            if (Double.isNaN(xs[i]) || Double.isNaN(ys[i]) || Double.isInfinite(xs[i]) || Double.isInfinite(ys[i])) {
                throw new IllegalArgumentException("InterpolatingTable point " + i + " is not a finite number");
            }
            if (i > 0 && !(xs[i] > xs[i - 1])) {
                throw new IllegalArgumentException("InterpolatingTable x values must be strictly increasing (row " + i + ")");
            }
        }
        this.xs = Arrays.copyOf(xs, xs.length);
        this.ys = Arrays.copyOf(ys, ys.length);
    }

    /** Builds a table from column {@code xColumn} and {@code yColumn} of a row-major array. */
    public static InterpolatingTable fromColumns(double[][] rows, int xColumn, int yColumn) {
        if (rows == null || rows.length < 2) {
            throw new IllegalArgumentException("Shot table needs at least 2 rows");
        }
        double[] x = new double[rows.length];
        double[] y = new double[rows.length];
        for (int i = 0; i < rows.length; i++) {
            if (rows[i] == null || rows[i].length <= Math.max(xColumn, yColumn)) {
                throw new IllegalArgumentException("Shot table row " + i + " is missing columns");
            }
            x[i] = rows[i][xColumn];
            y[i] = rows[i][yColumn];
        }
        return new InterpolatingTable(x, y);
    }

    public double minX() {
        return xs[0];
    }

    public double maxX() {
        return xs[xs.length - 1];
    }

    public double get(double x) {
        if (Double.isNaN(x)) return ys[0];
        if (x <= xs[0]) return ys[0];
        int last = xs.length - 1;
        if (x >= xs[last]) return ys[last];
        // Binary search for the segment [xs[lo], xs[lo + 1]] containing x.
        int lo = 0;
        int hi = last;
        while (hi - lo > 1) {
            int mid = (lo + hi) >>> 1;
            if (xs[mid] <= x) lo = mid; else hi = mid;
        }
        double t = (x - xs[lo]) / (xs[hi] - xs[lo]);
        return ys[lo] + t * (ys[hi] - ys[lo]);
    }
}
