package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Indicator {

    public enum Color {
        OFF(0),
        RED(0.277),
        ORANGE(0.333),
        AMBER(0.36),
        YELLOW(0.388),
        GREEN(0.5),
        BLUE(0.611), VIOLET(0.598),
        WHITE(1.0);
        public final double pos;
        Color(double pos) { this.pos = pos; }
    }

    private final Servo led;
    private Color current = null;

    public Indicator(HardwareMap hardwareMap) {
        led = hardwareMap.get(Servo.class, "a");
        set(Color.OFF);
    }

    public void set(Color c) {
        if (c == current) return;
        current = c;
        led.setPosition(c.pos);
    }

    /**
     * Smooth fade from RED (t = 0) through orange/yellow to GREEN (t = 1).
     * The light's position is its hue, so the in-between values are the in-between colors.
     */
    public void setRedToGreen(double t) {
        t = Math.max(0, Math.min(1, t));
        current = null; // not one of the named colors
        led.setPosition(Color.RED.pos + (Color.GREEN.pos - Color.RED.pos) * t);
    }
}
