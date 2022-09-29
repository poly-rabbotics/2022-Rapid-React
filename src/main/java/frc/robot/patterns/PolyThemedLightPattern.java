package frc.robot.patterns;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.helperClasses.*;

/**
 * Green and Gold to please our jackrabbit overlords.
 */
public class PolyThemedLightPattern implements LightPattern {
	// The light strip takes (255, 255, 255) to be the same as (1, 1, 1).
	private static final Color GREEN = new Color(0, 0.58, 0); 
	private static final Color GOLD = new Color(0.58, 0.27, 0);

	private boolean requestingReset = false;
	private double time = 0.0;
	private double speed;
	private Color[] pattern;

	@Override
	public Color[] getPattern(double time) {
		if (requestingReset) {
			final String message = "Timer reset request was not reciprocated. \"requestingReset\" was not of expected value \"false\".\nValue was:";
			SmartDashboard.putBoolean(message, requestingReset);
		}
		
		// If this.time is the same as thhe last time this method was called then theh pattern should not have changed.
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
		// Get a gradiant from the starting end to the middle, and from the middle to the other end. Subtraction is performed
		// rather than using the same length value to account for on lengths.
		final Color[] gradiantPartOne = ColorUtils.makeGradiant(GREEN, GOLD, pattern.length / 2);
		final Color[] gradiantPartTwo = ColorUtils.makeGradiant(GOLD, GREEN, pattern.length - (pattern.length / 2));

		int shift = (int)(speed * time) % pattern.length;

		// Shifts all colors in the pattern forward by shift, looping them back to the start of the array when they exceed 
		// the end pacman style.
		for (int i = 0; i < pattern.length; i++) {
			// Since we have two Color arrays, each being one half of our pattern, we need to get the Color that we are
			// currently setting some element in pattern to.
			Color currentColor = (i < gradiantPartOne.length) ? gradiantPartOne[i] : gradiantPartTwo[i - gradiantPartOne.length];

			// Now that we have the Color we want to set an element in pattern to, we need to get the index of that element.
			// Using modulo loops all indices around back to 0 if they exceed the bound of our array.
			pattern[(i + shift) % pattern.length] = currentColor;
		}

		requestingReset = shift >= pattern.length;
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
	public PolyThemedLightPattern(int length, double speed) {
		this.speed = speed;
		pattern = new Color[length];
		updatePattern();
	}
}
