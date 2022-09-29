package frc.robot.patterns;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.helperClasses.LightPattern;

/**
 * We can all use a rainbow even if it isnt June.
 */
public class RainbowLightPattern implements LightPattern {
	private static final int COLOR_SATURATION = 255;
	private static final int COLOR_BRIGHTNESS = 128;

	private boolean requestingReset = false;
	private double time = 0.0;
	private double speed;
	private Color[] pattern;

	@Override
	public Color[] getPattern(double time) {
		// After calling getPattern() LightRenderer should immediately make a call to getShouldResetTimer() and upon a return 
		// value of true immediately reset the timer. This means that if ever getPattern() is called while requestingReset is
		// true a non-fatal error has occured either in this class in LightRenderer.
		if (requestingReset) {
			final String message = "Timer reset request was not reciprocated. \"requestingReset\" was not of expected value \"false\".\nValue was:";
			SmartDashboard.putBoolean(message, requestingReset);
		}
		
		// If this.time is the same as the last time this method was called then theh pattern should not have changed.
		if (time == this.time)
			return pattern;

		// When the time changes, record it in the this.time feild and update the pattern Color array feild.
		this.time = time;
		updatePattern();
		return pattern;
	}

	@Override
	public boolean getShouldResetTimer() {
		return requestingReset;
	}

	@Override
	public int getPatternLength() {
		return pattern.length;
	}

	// updates the Color[] pattern private feild to account for a change in the private feild double time.
	private void updatePattern() {		
		final double hueShift = time * speed;

		for (var i = 0; i < pattern.length; i++) {
            final double hue = (hueShift + (i * 180 / pattern.length)) % 180;
            pattern[i] = Color.fromHSV((int)hue, COLOR_SATURATION, COLOR_BRIGHTNESS);
        }

		requestingReset = hueShift >= 180.0;
	}

	/**
	 * Creates a new {@link RainbowLightPattern}.
	 * 
	 * @param length
	 * The number of colors to calculate a rainbow for.
	 * 
	 * @param speed
	 * The speed at which to change over time with using a {@link LightRenderer}.
	 */
	public RainbowLightPattern(int length, double speed) {
		this.speed = speed;
		pattern = new Color[length];
		updatePattern();
	}
}
