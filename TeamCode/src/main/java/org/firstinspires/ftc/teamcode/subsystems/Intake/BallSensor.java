package org.firstinspires.ftc.teamcode.subsystems.Intake;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Color sensor on the transfer. Tells us if a ball is in front of the sensor and
 * whether it is POLLEN. Tune everything below live in FTC Dashboard (the hue/distance
 * values are printed in telemetry via {@link #debug()}).
 */
@Config
public class BallSensor {
    public static String NAME = "colorsensor";
    public static float GAIN = 2;
    /** Ball counts as present when closer than this (cm). Needs a REV Color Sensor V3 (has distance). */
    public static double PRESENT_CM = 2;
    /** POLLEN hue range in degrees (0-360). TODO: read the hue of a pollen ball in debug() and set these. */
    public static double POLLEN_HUE_MIN = 45, POLLEN_HUE_MAX = 75; // yellow ~60

    /** Yellow must be seen continuously this long to count as 3 pollen (a ball passing by is ignored). */
    public static double HOLD_MS = 250;

    private final NormalizedColorSensor sensor;
    private long yellowSince = -1;
    private boolean full = false;
    private double cm = 999, hue = 0;
    private boolean present = false, pollen = false;

    public BallSensor(HardwareMap hardwareMap) {
        sensor = hardwareMap.get(NormalizedColorSensor.class, NAME);
        sensor.setGain(GAIN);
    }

    /** Call once per loop. */
    public void update() {
        sensor.setGain(GAIN);
        NormalizedRGBA c = sensor.getNormalizedColors();
        cm = sensor instanceof DistanceSensor ? ((DistanceSensor) sensor).getDistance(DistanceUnit.CM) : 999;
        hue = JavaColor(c);
        present = cm < PRESENT_CM;
        pollen = present && hue >= POLLEN_HUE_MIN && hue <= POLLEN_HUE_MAX;
        long now = System.currentTimeMillis();
        if (!pollen) yellowSince = -1;
        else if (yellowSince < 0) yellowSince = now;
        full = yellowSince >= 0 && now - yellowSince >= HOLD_MS;
    }

    private static float JavaColor(NormalizedRGBA c) {
        float[] hsv = new float[3];
        Color.colorToHSV(c.toColor(), hsv);
        return hsv[0];
    }

    public boolean hasBall() { return present; }
    /** 3 pollen loaded: yellow steadily in front of the sensor. */
    public boolean isFull() { return full; }
    public boolean hasPollen() { return pollen; }
    /** A ball is there but it is NOT pollen (nectar) -- driver should know. */
    public boolean hasWrongBall() { return present && !pollen; }

    public String debug() {
        return String.format("cm=%.1f hue=%.0f ball=%b yellow=%b full=%b", cm, hue, present, pollen, full);
    }
}
