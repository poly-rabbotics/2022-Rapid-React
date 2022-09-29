package frc.robot.patterns;

import java.util.Random;
import edu.wpi.first.wpilibj.util.Color;

import frc.robot.subsystems.helperClasses.LightPattern;

public class RandomPattern implements LightPattern {
    Color[] pattern;
    boolean requestingReset = false;
    int counter = 0;
    double time = 0;
    double speed = 0;
    
    public RandomPattern(int length, double speed) {
        pattern = new Color[length];
        this.speed = speed;
        
        for(int i = 0; i < length; i++) {
            pattern[i] = Color.fromHSV(new Random().nextInt(361), 255, 255);
        }
    }
    
    public Color[] getPattern(double time) {
        this.time = time;
        updatePattern();
        return pattern;
    }
    
    private void updatePattern() {
        int shift = (int)(speed * time) % pattern.length;

        for(int i = 0; i < pattern.length; i++) {
            Color currentColor = pattern[i];
            pattern[(i+shift) % pattern.length] = currentColor;
        }

        requestingReset = shift >= pattern.length;

        /*if(counter == interval) {
            for(int i = 0; i < pattern.length; i++) {
                if(i == 0) pattern[i] = pattern[pattern.length];
                else {
                    pattern[i] = pattern[i-1];
                }
            }

            counter = 0;
        }
        else counter++;*/
    }
    
    public int getPatternLength() {
        return pattern.length;
    }
    
    public boolean requestingReset() {
        return requestingReset;
    }
    
    public boolean getShouldResetTimer() {
        return false;
    }
    
}
