package frc.robot.patterns;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.helperClasses.LightPattern;
/**
 * A class for making a blinking color pattern, either cycling through an
 * array of colors, or using just one color and blinking on and off.
 */
public class Blink implements LightPattern {
	private static final Color DEFAULT_COLOR = new Color(1.0, 1.0, 1.0);
	private static final Color LIGHTS_OFF = new Color(0.0, 0.0, 0.0);
	private static final double DEFAULT_SPEED = 2.0;

	private Color[] pattern = new Color[1];
	private Color[] colors = new Color[] { DEFAULT_COLOR, LIGHTS_OFF };
	private int currentIndex = 0;
	private double speed = DEFAULT_SPEED;
	private boolean requestingReset = false;

	@Override
	public Color[] getPattern(double time) {
		updatePattern(time);
		return pattern;
	}

	@Override
	public boolean getShouldResetTimer() {
		return requestingReset;
	}

	@Override
	public int getPatternLength() {
		// Since this pattern is just a blinking solid color its length is always 1.
		return 1;
	}

	private void updatePattern(double time) {
		// Here speed * time is equivilant to the index that we should be on
		// in the colors array, so if it exceeds the length of that array we
		// have run our pattern to completion and therefore should request a reset.
		requestingReset = time * speed >= colors.length;

		pattern[0] = colors[currentIndex];

		// By incrementing our current color like this it is very possible that 
		// we will spend an extra iteration of this pattern on the first color of
		// the colors array. This is neglegable for something like this so long as
		// it is looped fast enough, looping at the standard 50 hz or more is more
		// than sufficiant.
		if (time * speed >= 1.0) {
			currentIndex++;
			currentIndex %= colors.length;
		}
	}

	/**
	 * Creates a default instance of the {@link Blink} class.
	 */
	public Blink() { 
		updatePattern(0.0);
	}

	/**
	 * Creates an instace of the {@link Blink} class with the given speed.
	 * Here the speed parameter will directly be the number of times to change
	 * color per second.
	 */
	public Blink(double speed) {
		this.speed = speed;
		updatePattern(0.0);
	}

	/**
	 * Creates an instance of the {@link Blink} class that flashes between off
	 * and the given color.
	 */
	public Blink(Color color) {
		colors = new Color[] { color, LIGHTS_OFF };
		updatePattern(0.0);
	}
	
	/**
	 * Creates an instance of the {@link Blink} class that flashes between off
	 * and the given color at the given speed. Here the speed parameter 
	 * will directly be the number of times to change color per second.
	 */
	public Blink(Color color, double speed) {
		colors = new Color[] { color, LIGHTS_OFF };
		this.speed = speed;
		updatePattern(0.0);
	}

	/**
	 * Creates an instance of the {@link Blink} class that flashes from one color 
	 * in the given arrray to the next by order of element index. 
	 */
	public Blink(Color[] colors) {
		this.colors = colors;
		updatePattern(0.0);
	}

	/**
	 * Creates an instance of the {@link Blink} class that flashes from one color 
	 * in the given arrray to the next by order of element index at the given speed. 
	 * Here the speed parameter will directly be the number of times to change
	 * color per second.
	 */
	public Blink(Color[] colors, double speed) {
		this.colors = colors;
		this.speed = speed;
		updatePattern(0.0);
	}
}
