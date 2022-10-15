package frc.robot.subsystems.autoModes;

import edu.wpi.first.wpilibj.*;
import frc.robot.*;

public class modeFive extends Thread {
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
   	    	Robot.drive.moveByInches(0, 1.5, 21);
        	Robot.drive.turnByDegrees(1.5, 3, 47);
        	Robot.drive.moveByInches(3, 4.5, 54);
        	Robot.intake.autoRun(4, 6, -0.8);
        	Robot.intake.autoRun(6, 17, 0);
        	Robot.conveyor.autoRun(5, 6, 0.4);
        	Robot.conveyor.autoRun(6, 7, 0);
        	Robot.drive.turnByDegrees(6, 7, -20);
        	Robot.drive.moveByInches(7, 8.5, -103);
        	Robot.drive.turnByDegrees(8.5, 9.5, -51);
        	Robot.drive.moveByInches(9.5, 10.5, -23);
        	Robot.shooter.autoRun(9.5, 11, -0.8);
        	Robot.conveyor.autoRun(10.5, 12.5, 0.8);
        	Robot.drive.moveByInches(12.5, 13, 23);
        	Robot.drive.turnByDegrees(13, 14, 60);
        	Robot.drive.moveByInches(14, 16, 223);
        	Robot.drive.turnByDegrees(16, 17, -20);
        	Robot.drive.moveByInches(17, 18, 41);
        	Robot.intake.autoRun(17, 20, -0.8);
        	Robot.conveyor.autoRun(18, 21, 0.3);
        	Robot.conveyor.autoRun(21, 22, 0);
        	Robot.drive.turnByDegrees(19, 20, 43);
        	Robot.drive.moveByInches(20, 21, -35);
        	Robot.drive.turnByDegrees(21, 22, -28);
       	 	Robot.drive.moveByInches(22, 24, -232);
        	Robot.drive.turnByDegrees(24, 25, -50);
        	Robot.drive.moveByInches(25, 26, -26);
        	Robot.shooter.autoRun(25, 28, -0.8);
        	Robot.conveyor.autoRun(26, 28, 0.8);
		}

		terminated = true;
	}
}

