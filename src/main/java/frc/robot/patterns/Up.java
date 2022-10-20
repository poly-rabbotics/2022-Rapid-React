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
        pattern = new Color[length];
        this.speed = speed;
        this.length = length;
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

        /*pattern[(int)position] = Color.fromHSV(hue, 255, 255);
        for(int i = 0; i < trailLength; i++){
            pattern[(int)position-i-1] = Color.fromHSV(hue, 255, (int)(255 - (255-(255/trailLength)*(i+1))));
            pattern[10] = Color.fromHSV(hue, 255, 255);
        }*/
    }

    @Override
    public Color[] getPattern(double time) {
        this.time = time;
        updatePattern();
        //requestingReset = position >= pattern.length + trailLength;
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
