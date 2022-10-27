package frc.robot.patterns;

import frc.robot.subsystems.helperClasses.LightPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

/**
 * A class for producing a solid color on light strips.
 */
public class Up implements LightPattern {
    Color[] pattern;
    int length;
    double time;
    double speed;
    double rainbowSpeed;
    int hue = 68;
    int trailLength;
    boolean requestingReset = false;
    boolean rainbowMode = false;

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
    public Up(double speed, int length, double rainbowSpeed, int trailLength) {
        pattern = new Color[length];
        this.speed = speed;
        this.length = length;
        this.trailLength = trailLength;
        this.rainbowSpeed = rainbowSpeed;

        //Rainbow mode is only turned on if speed is given
        if(rainbowSpeed > 0) rainbowMode = true;
    }

    public Up(double speed, int length, int hue, int trailLength) {
        pattern = new Color[length];
        this.speed = speed;
        this.length = length;
        this.hue = hue;
        this.trailLength = trailLength;
    }

    private void updatePattern() {
        int position = (int)((time * speed) % pattern.length);
        int increment = 255 / trailLength;

        if(rainbowMode)
            hue = (int)((time * rainbowSpeed) % 180);

        if (position >= pattern.length)
            requestingReset = true;
        else
            requestingReset = false;

        

        for(int i = 0; i < pattern.length; i++) {
            pattern[i] = Color.fromHSV(0, 0, 0);
        }

        for (int i = 0; i < pattern.length; i++) {

            if (i == position) {
                for(int j = 0; j < trailLength; j++) {
                    if(i - j >= 0)
                        pattern[i-j] = Color.fromHSV(hue, 255, 255 - (increment*j));
                    else 
                        pattern[pattern.length + (i-j)] = Color.fromHSV(hue, 255, 255 - (increment*j));
                }
                //increment += 255 / trailLength;
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
