package frc.robot.patterns;

import frc.robot.subsystems.helperClasses.LightPattern;
import edu.wpi.first.wpilibj.util.Color;

/**
 * A class for producing a solid color on light strips.
 */
public class SolidColor implements LightPattern {
    Color[] pattern;

    /**
     * Creates a new {@link SolidColor} pattern.
     *
     * @param r
     * This {@link SolidColor}'s red component.
     *
     * @param g
     * This {@link SolidColor}'s green component.
     *
     * @param b
     * This {@link SolidColor}'s blue component.
     */
    public SolidColor(int r, int g, int b) {
        pattern = new Color[] {
            new Color(r/255, g/255, b/255)
        };
    }

    @Override
    public Color[] getPattern(double time) {
        return pattern;
    }

    @Override
    public int getPatternLength() {
        return pattern.length;
    }

    @Override
    public boolean getShouldResetTimer() {
        return true;
    }
}
