package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Gate servo between the feeder and the launcher: closed holds balls back, open lets them
 * through. {@link #isOpen()} turns true GATE_OPEN_DELAY_SEC after the open command, so the feeder
 * does not push a ball into a gate that is still moving.
 */
public final class Gate {
    private final Servo servo;
    private boolean wantOpen;
    private boolean started;
    private double openedAt = Double.NaN;

    public Gate(HardwareMap hardwareMap) {
        // No command here: the servo stays unpowered until the first update() after START.
        servo = hardwareMap.get(Servo.class, ShooterConstants.GATE_SERVO_NAME);
    }

    /** Call once per loop: true opens the gate, false closes it. The first call commands it. */
    public void update(boolean open) {
        if (started && open == wantOpen) return;
        started = true;
        wantOpen = open;
        openedAt = open ? System.nanoTime() * 1e-9 : Double.NaN;
        servo.setPosition(open ? ShooterConstants.GATE_OPEN_POSITION : ShooterConstants.GATE_CLOSED_POSITION);
    }

    /** True once the gate has been open for GATE_OPEN_DELAY_SEC. */
    public boolean isOpen() {
        return wantOpen && System.nanoTime() * 1e-9 - openedAt >= ShooterConstants.GATE_OPEN_DELAY_SEC;
    }

    public boolean wantsOpen() {
        return wantOpen;
    }

    public void close() {
        update(false);
    }
}
