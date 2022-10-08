package frc.robot.patterns;

import frc.robot.subsystems.helperClasses.*;
import edu.wpi.first.wpilibj.util.Color;

/**
 * A class for producing a solid color on light strips.
 */
public class Up implements LightPattern {
    private Color[] pattern;
    private double time;
    private double speed;
    private int hue;
    private int trailLength;
    private boolean requestingReset = false;

    /**
     * Creates a new {@link Up} pattern.
     *
     * @param r
     * This {@link Up}'s red component.
     *
     * @param g
     * This {@link Up}'s green component.
     *
     * @param b
     * This {@link Up}'s blue component.
     */
    public Up(double speed, int length, int hue, int trailLength) {
        pattern = new Color[length];

		this.speed = speed;
        this.hue = hue;
        this.trailLength = trailLength;
    }

    private void updatePattern() {
        int position = (int)(time * speed);
        int increment = 255 / trailLength;

        if (position >= pattern.length)
            requestingReset = true;
        else
            requestingReset = false;

        for (int i = 0; i < pattern.length + trailLength; i++) {
            if (i == position && increment <= 255) {
                pattern[i % pattern.length] = Color.fromHSV(hue, 255, increment);
                increment += 255 / trailLength;
                position++;
            } else {
                pattern[i % pattern.length] = new Color(0.0, 0.0, 0.0);
            }
        }
    }

    @Override
    public Color[] getPattern(double time) {
        this.time = time;
        updatePattern();
        return pattern;
    }

    @Override
    public int getPatternLength() {
        return pattern.length;
    }

    @Override
    public boolean getShouldResetTimer() {
        return requestingReset;
    }
}
