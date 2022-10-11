package frc.robot.subsystems.autoModes;

import frc.robot.*;
import edu.wpi.first.wpilibj.*;

public class modeSix extends Thread {
	private Timer timer;
	private boolean terminationRequested;
	private boolean terminated = false;

	public void requestTermination() {
		terminationRequested = true;
	}

	public boolean getTerminationStatus() {
		return terminated;
	}

	@Override
	public void run() {
		timer = new Timer();
		timer.start();

		while (timer.get() < 15.0 && !terminationRequested) {
			Robot.intake.deployIntake(0, 1, true);
			Robot.drive.moveByInches(0, 2, -55.5);
			Robot.intake.autoRun(2, 4, -0.8);
			
			// Halt intake until next ball:
			Robot.intake.autoRun(4, 18, 0.0);

			Robot.drive.moveByInches(4, 6, 55.5);
			Robot.drive.turnByDegrees(6, 7, 21);
			Robot.drive.moveByInches(7, 9, 12);
			Robot.shooter.autoRun(9, 12, -4600);
			Robot.drive.moveByInches(12, 13, -8.54);
			Robot.drive.turnByDegrees(13, 15, 88.4);
			Robot.drive.moveByInches(15, 18, -81.3);
			Robot.intake.autoRun(18, 20, -0.8);
			
			// halt intake until next pickup
			Robot.intake.autoRun(20, 28, 0.0);
			
			Robot.drive.moveByInches(20, 26, -159.9);
			Robot.intake.autoRun(26, 28, -0.8);

			//halt intake:
			Robot.intake.autoRun(28, 50, 0.0);

			Robot.drive.moveByInches(28, 38, 159.9 + 81.3);
			Robot.drive.turnByDegrees(38, 40, -88.4);
			Robot.drive.moveByInches(40, 41, 8.54);
			Robot.shooter.autoRun(41, 44, -4600);
		}

		terminated = true;
	}
}

