package frc.robot.subsystems.autoModes;

import edu.wpi.first.wpilibj.*;
import frc.robot.*;

public class modeThree extends Thread {
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
        	Robot.intake.deployIntake(0, 1, true);
        	Robot.drive.resetEncodersCall(0, 0.05);
        	Robot.intake.autoRun(0, 15, -0.85);
        	Robot.shooter.autoRun(0, 3, -4800);
        	Robot.conveyor.autoRun(2, 3, 1);
        	Robot.shooter.autoRun(3, 3.1, 0);
        	Robot.conveyor.autoRun(3, 3.1, 0);
        	Robot.drive.goToEncCountsTurn(0 +1.5, 1+1.5, 6000);
        	Robot.drive.resetEncodersCall(1+1.5, 1.01+1.5);
        	Robot.drive.goToEncCounts(1.01+1.5,3+1.5,90000);
        	Robot.drive.resetEncodersCall(3+1.5, 3.01+1.5);
        	Robot.conveyor.autoRun(3+1.5, 5+1.5, 0.3);
        	Robot.conveyor.autoRun(5+1.5, 5.1+1.5, 0);
        	Robot.drive.goToEncCounts(3.01+1.5, 4.5+1.5, -80000);
        	Robot.drive.resetEncodersCall(4.5+1.5, 4.51+1.5);
        	Robot.drive.goToEncCountsTurn(4.51+1.5, 5.5+1.5, -25000);
        	Robot.drive.resetEncodersCall(5.5+1.5, 5.51+1.5);
        	Robot.drive.goToEncCounts(5.51+1.5, 7.5+1.5, 130000);
        	Robot.drive.resetEncodersCall(7.5+1.5, 7.51+1.5);
        	Robot.drive.goToEncCounts(7.51+1.5, 9.5+1.5, -120000);
        	Robot.drive.resetEncodersCall(9.5+1.5, 9.51+1.5);
        	Robot.drive.goToEncCountsTurn(9.51+1.5 ,10.5+1.5, 20000);
        	Robot.shooter.autoRun(10.5+1.5, 15+1.5, -4700);
        	Robot.conveyor.autoRun(12.2+1.5, 15+1.5, 0.8);
        	Robot.drive.resetEncodersCall(10.5+1.5, 10.51+1.5);
        	Robot.drive.goToEncCounts(10.51+1.5, 12.5+1.5, -30000);
		}

		terminated = true;
	}
}

