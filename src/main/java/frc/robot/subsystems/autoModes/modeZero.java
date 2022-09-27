package frc.robot.subsystems.autoModes;

import edu.wpi.first.wpilibj.*;
import frc.robot.*;

public class modeZero extends Thread {
	private Timer timer;
	private boolean terminationRequested;
	private boolean terminated = false;

	/**
	 * Requests termination so that when the current loop
	 * iteration completes the loop will no longer iterate.
	 */
	public void requestTermination() {
		terminationRequested = true;
	}
	
	/**
	 * Returns true if the thread's loop has terminated.
	 * Not that just because this method returns false
	 * does not necassarily mean it has started it loop,
	 * and likewise a value of true does not necassarily
	 * indicate that the run() method has exited.
	 */
	public boolean getTerminationStatus() {
		return terminated;
	}

	@Override
	public void run() {
		timer = new Timer();
		timer.start();

		// Loop terminates once this thread has been running for 15 seconds.
		// Loop will also terminate is requestTermination() is called.
		while (timer.get() < 15.0 && !terminationRequested) {
			Robot.shooter.autoRun(0, 15, 0);
			Robot.conveyor.autoRun(0, 15, 0);
			Robot.intake.autoRun(0, 15, 0);
		}

		terminated = true;
	}
}

