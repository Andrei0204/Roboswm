package org.firstinspires.ftc.teamcode.shooter;

/**
 * Tracks which CELL of our HIVE is up, so it can be targeted (G417). The up CELL swaps after
 * every TIP, and the robot then has to shoot from the other half of the field.
 *
 * <p>The operator reports each TIP with a button. To automate it, feed vision observations into
 * {@link #setUpCell}: the CELL whose AprilTag cluster ({@link BioBuzzField#cellTagIds}) reads a
 * roll under 90 deg is the up one. Firing pauses briefly after a reported TIP, because the manual
 * advises not disturbing a HIVE while it is tipping.
 */
public final class HiveState {
    private HiveCell upCell;
    private double pauseUntil = Double.NEGATIVE_INFINITY;
    private int tips;

    public HiveState(Alliance alliance) {
        reset(alliance);
    }

    /** Back to the match-start state for {@code alliance}. */
    public void reset(Alliance alliance) {
        upCell = BioBuzzField.startUpCell(alliance);
        pauseUntil = Double.NEGATIVE_INFINITY;
        tips = 0;
    }

    public HiveCell upCell() {
        return upCell;
    }

    public void setUpCell(HiveCell cell) {
        upCell = cell;
    }

    /** The operator saw our HIVE tip: target the other CELL and pause firing while it settles. */
    public void reportTip(double nowSec) {
        upCell = upCell.other();
        pauseUntil = nowSec + ShooterConstants.HIVE_TIP_PAUSE_SEC;
        tips++;
    }

    /** Undo a mistaken TIP report. */
    public void undoTip() {
        upCell = upCell.other();
        pauseUntil = Double.NEGATIVE_INFINITY;
        if (tips > 0) tips--;
    }

    public boolean isPaused(double nowSec) {
        return nowSec < pauseUntil;
    }

    public int tips() {
        return tips;
    }
}
