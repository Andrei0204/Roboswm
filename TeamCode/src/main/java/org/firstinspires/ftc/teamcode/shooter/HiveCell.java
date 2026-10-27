package org.firstinspires.ftc.teamcode.shooter;

/** The two CELLs of an alliance's HIVE seesaw. Only the CELL that is currently up is a legal target (G417). */
public enum HiveCell {
    /** CELL on the audience side of the field center. */
    AUDIENCE,
    /** CELL on the rear (scoring-table wall) side of the field center. */
    REAR;

    public HiveCell other() {
        return this == AUDIENCE ? REAR : AUDIENCE;
    }
}
