package frc.robot.patterns;

import frc.robot.subsystems.helperClasses.LightPattern;
import edu.wpi.first.wpilibj.util.Color;

/**
 * A class for producing a solid color on light strips.
 */
public class Up implements LightPattern {
    Color[] pattern;
    int length;
    double time;
    double speed;
    int hue;
    double position = 0;
    int trailLength;
    boolean requestingReset = false;

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
        this.speed = speed;
        this.length = length;
        this.hue = hue;
    }

    private void updatePattern() {
        position = (time * speed) % pattern.length;
        for(int i = 0; i < pattern.length; i++) {
            pattern[i] = Color.fromHSV(hue, 255, 255);
            for(int j = 0; j < trailLength; i++) {
                pattern[i-j] = Color.fromHSV(hue, 255, 255 - ((trailLength/255)*j));
            }
        }
    }

    @Override
    public Color[] getPattern(double time) {
        this.time = time;
        updatePattern();
        requestingReset = position >= pattern.length;
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
