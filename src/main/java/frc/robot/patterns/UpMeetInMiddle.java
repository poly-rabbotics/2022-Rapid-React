package frc.robot.patterns;

import frc.robot.subsystems.helperClasses.LightPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

/**
 * A class for producing a solid color on light strips.
 */
public class UpMeetInMiddle implements LightPattern {
    Color[] pattern;
    int[] rainbowHueArr;
    int length;
    double time;
    double speed;
    double rainbowSpeed;
    int hue;
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
    public UpMeetInMiddle(double speed, int length, int trailLength, boolean rainbowMode) {
        pattern = new Color[length];
        rainbowHueArr = new int[length];
        this.speed = speed;
        this.length = length;
        this.trailLength = trailLength;
        this.rainbowMode = rainbowMode;
    }

    public UpMeetInMiddle(double speed, int length, int hue, int trailLength) {
        pattern = new Color[length];
        this.speed = speed;
        this.length = length;
        this.hue = hue;
        this.trailLength = trailLength;
    }

    private void updatePattern() {
        int position = (int)((time * speed) % pattern.length);
        int oppositePosition = pattern.length - position;
        int increment = 255 / trailLength;

        //resetting pattern
        if (position >= pattern.length)
            requestingReset = true;
        else
            requestingReset = false;

        //Set all leds to blank
        for(int i = 0; i < pattern.length; i++) {
            pattern[i] = Color.fromHSV(0, 0, 0);
            rainbowHueArr[i] = (int)((180/pattern.length) * i);
        }

        //Set trails
        for (int i = 0; i < pattern.length/2; i++) {

            if (i == position) {
                for(int j = 0; j < trailLength; j++) {
                    //SET RIGHT TRAIL
                    if(i - j >= 0) {
                        hue = rainbowHueArr[position - j];
                        pattern[position - j] = Color.fromHSV(hue, 255, 255 - (increment * j));
                    } else {
                        hue = rainbowHueArr[(pattern.length/2) + (position - j)];
                        pattern[(pattern.length/2) + (position - j)] = Color.fromHSV(hue, 255, 255 - (increment * j));
                    }

                    //SET LEFT TRAIL
                    if(oppositePosition + j <= pattern.length) {
                        hue = rainbowHueArr[oppositePosition + j];
                        pattern[oppositePosition + j] = Color.fromHSV(hue, 255, 255 - (increment * j));
                    } else {
                        hue = rainbowHueArr[(pattern.length/2) + ((oppositePosition - pattern.length) + j)];
                        pattern[(pattern.length/2) + ((oppositePosition - pattern.length) + j)] = Color.fromHSV(hue, 255, 255 - (increment * j));
                    }
                }
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
