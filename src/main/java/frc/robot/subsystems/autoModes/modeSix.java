package frc.robot.subsystems.autoModes;

import frc.robot.*;
import java.util.TimerTask;
import edu.wpi.first.wpilibj.*;

public class modeSix extends Thread {
	private static final long ITERATION_MS = 10;
	private Timer timer;
	private boolean terminationRequested;
	private boolean terminated = false;

	/**
	 * Request the termination of the autonomous mode prematurely.
	 * Note that this will wait for the current iteration to complete,
	 * and that even if this autonomous mode has terminated, its actual
	 * thread may not have.
	 */
	public void requestTermination() {
		terminationRequested = true;
	}

	/**
	 * Gets whether or not this autonomous thread has terminated.
	 * Keep in mind that this does not mean the thread has exited.
	 */
	public boolean getTerminationStatus() {
		return terminated;
	}

	@Override
	public void run() {
		var taskTimer = new java.util.Timer();
		timer = new Timer();
		timer.start();
		
		// Use java's inbuilt timer class to schedual java's TimerTask's
		// run() method at a fixed rate using ITERATION_MS.
		taskTimer.schedule(new TimerTask() {
			public void run() {
				// Checks if autonomous period is over, or termination has
				// been requested.
				if (timer.get() >= 15.0 || terminationRequested) {
					// The thread will remain active as is.
					terminated = true;
					return;
				}

				// Run the actual autonomous.
				runAutoSteps();
			}
		}, 0, ITERATION_MS);

		while (timer.get() < 15.0 && !terminationRequested) {
		}

		terminated = true;
	}

	// This actualy runs autonomous, everything else is to make
	// this work, good luck.
	public void runAutoSteps() {
		Robot.intake.deployIntake(0, 1, true);
		Robot.drive.moveByInches(0, 2, -55.5);
		Robot.intake.autoRun(2, 4, -0.8);
		
		// Halt intake until next ball:
		Robot.intake.autoRun(4, 18, 0.0);
		Robot.conveyor.autoRun(4, 6, 0.4);
		Robot.conveyor.autoRun(6, 7, 0.0);
		Robot.drive.moveByInches(4, 6, 55.5);
		Robot.drive.turnByDegrees(6, 7, 21);
		Robot.drive.moveByInches(7, 9, 12);
		Robot.conveyor.autoRun(7, 12, 0.4);
		Robot.shooter.autoRun(9, 12, -4600);
		Robot.conveyor.autoRun(12, 13, 0.0);
		Robot.drive.moveByInches(12, 13, -8.54);
		Robot.drive.turnByDegrees(13, 15, 88.4);
		Robot.drive.moveByInches(15, 18, -81.3);
		Robot.intake.autoRun(18, 20, -0.8);
		Robot.conveyor.autoRun(20, 22, 0.4);
		Robot.conveyor.autoRun(22, 23, 0.0);
		
		// halt intake until next pickup
		Robot.intake.autoRun(20, 28, 0.0);
		
		Robot.drive.moveByInches(20, 26, -159.9);
		Robot.intake.autoRun(26, 28, -0.8);
		Robot.conveyor.autoRun(28, 30, 0.4);
		Robot.conveyor.autoRun(30, 31, 0.0);

		//halt intake:
		Robot.intake.autoRun(28, 50, 0.0);

		Robot.drive.moveByInches(28, 38, 159.9 + 81.3);
		Robot.drive.turnByDegrees(38, 40, -88.4);
		Robot.drive.moveByInches(40, 41, 8.54);
		Robot.conveyor.autoRun(41, 44, 0.4);
		Robot.shooter.autoRun(41, 44, -4600);
		Robot.conveyor.autoRun(44, 45, 0.0);
	}
}

