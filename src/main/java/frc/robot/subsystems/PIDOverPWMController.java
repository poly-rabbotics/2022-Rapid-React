package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;

/**
 * Implements a PID loop ontop an Encoder and PWM Conmtroller.
 */
public class PIDOverPWMController implements Runnable {
	private static final int ITERATION_INTERVAL = 10;

	private PWMMotorController pwmMotorController;
	private Encoder encoder;
	private Timer timer;

	private boolean running = false;

	// Make the setpoint the same as the encoder count to prevent any movement on initialization.
	private int setPoint = encoder.get();
	
	// please dont ever use these defaults, these are here for the strict purpose of preventing a reference to null.
	private double kp = 0.1;
	private double ki = 0.0;
	private double kd = 0.0;

	private double deltaTime;
	private double previousError = 0.0;
	private double accumulativeError = 0.0;

	// This value could be made a reasonable default with some testing on a commonly used encoder.
	private double degreeToCountRatio = 1.0;

	//#region property setters

	/** 
	 * Sets the degree to count ratio. (degree / count) 
	 */
	public void setDegreeToCountRatio(double degreeToCountRatio) {
		this.degreeToCountRatio = degreeToCountRatio;
	}

	/** 
	 * Sets the setPoint angle in encoder counts. 
	 */
	public void setSetPoint(int setPoint) {
		this.setPoint = setPoint;
	}

	/** 
	 * Sets the setPoint angle in degrees. 
	 */
	public void setSetPointDegrees(double degrees) {
		setPoint = (int)(degrees * degreeToCountRatio);
	}

	/**
	 * Sets the P component of this PID loop.
	 */
	public void setPComponent(double kp) {
		this.kp = kp;
	}

	/**
	 * Sets the I component of this PID loop.
	 */
	public void setIComponent(double ki) {
		this.ki = ki;
	}

	/**
	 * Sets the D component of this PID loop.
	 */
	public void setDComponent(double kd) {
		this.kd = kd;
	}

	//#endregion

	/**
	 * Finishes this {@link PIDOverPWMController}'s current loop iteration and then exits.
	 */
	public void stopRunning() {
		running = false;
	}

	/**
	 * Creates a new {@link PIDOverPWMController}.
	 * 
	 * @param pwmMotorController
	 * The motor controller to implement PID on top of. 
	 * 
	 * @param encoder
	 * The encoder attached to that motor.
	 */
	public PIDOverPWMController(PWMMotorController pwmMotorController, Encoder encoder) {
		this.pwmMotorController = pwmMotorController;
		this.encoder = encoder;

		timer.reset();
		timer.start();
	}

	/**
	 * Creates a new {@link PIDOverPWMController}.
	 * 
	 * @param pwmMotorController
	 * The motor controller to implement PID on top of. 
	 * 
	 * @param encoder
	 * The encoder attached to that motor.
	 * 
	 * @param degreeToCountRatio
	 * The degree to count ratio of the passed in encoder.
	 */
	public PIDOverPWMController(PWMMotorController pwmMotorController, Encoder encoder, double degreeToCountRatio) {
		this.degreeToCountRatio = degreeToCountRatio;
		this.pwmMotorController = pwmMotorController;
		this.encoder = encoder;

		timer.reset();
		timer.start();
	}

	/**
	 * Runs this {@link PIDOverPWMController}. Note that this will block the current thread indefinitely. 
	 * See <code>runNonBlocking()</code> method.
	 */
	public void run() {
		running = true;

		while (running) {
			runNonBlocking();
			
			// sleeps the thread for the remaining interval time.
			try { Thread.sleep(ITERATION_INTERVAL - (long)(timer.get() / 1000)); }
			catch (InterruptedException e) { /* This thread shouldn't be interupted but if it is we just skip waiting */ }
		}
	}

	/**
	 * Runs this {@link PIDOverPWMController}.
	 */
	public void runNonBlocking() {
		deltaTime = timer.get();
		timer.reset();

		// Sets the PWM controllers output power, between -1.0 and 1.0
		pwmMotorController.set(calculatePID());
	}

	// Calculates output to controller and updates all feilds.
	private double calculatePID() {
		int error = setPoint - encoder.get();

		accumulativeError += (double)error * deltaTime;
		double derivativeOfError = ((double)error - previousError) / deltaTime;

		double p = error * kp;
		double i = accumulativeError * ki;
		double d = derivativeOfError * kd;
		
		double componentSum = p + i + d;

		// Clamp componentSum between -1.0 and 1.0 to account for saturation.
		componentSum = componentSum >= 1.0 ? 1.0 : componentSum; 
		componentSum = componentSum <= -1.0 ? -1.0 : componentSum; 

		// set error for next iteration.
		previousError = error;
		return componentSum;
	}
}