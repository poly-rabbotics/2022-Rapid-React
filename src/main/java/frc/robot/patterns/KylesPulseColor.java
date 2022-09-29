package frc.robot.patterns;

import frc.robot.subsystems.helperClasses.LightPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Timer;
/**
 * A class for producing a solid color on light strips.
 */
public class KylesPulseColor implements LightPattern {
    Color[] pattern;

    double val = 1;
    int hue;
    boolean requestingReset = false;
    boolean reverse = false;
    double time;
    double interval = 0;

    Timer Timer = new Timer();

    public KylesPulseColor(int hue, double interval) {
        this.hue = hue;
        this.interval = (interval/2)/200;

        pattern = new Color[] {
            Color.fromHSV(hue, 255, (int)val)
        };

        Timer.start();
    }

    @Override
    public Color[] getPattern(double time) {
        this.time = time;
        updatePattern();
        return pattern;
    }

    private void updatePattern() {

        if(time >= interval) {
            requestingReset = true;

            if(!reverse) val++;
            else if(reverse) val--;

            if(val >= 200) {
                val = 200;
                reverse = !reverse;
            } else if(val <= 0) {
                val = 0;
                reverse = !reverse;
            }

            pattern = new Color[] {
                Color.fromHSV(hue, 255, (int)val)
            };
        }
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
