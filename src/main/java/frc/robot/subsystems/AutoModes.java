// Now make some changes!

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.*;
import frc.robot.subsystems.autoModes.*;
import frc.robot.Controls.MechanismsJoystick;

public class AutoModes {
	private Thread autoThread; 
	private static int selectedMode = 0;

	// This array contains an instance of each possible autonomous mode,
	// null is used in place of a mode that has not been implemented.
	private static Thread[] availableModes = {
		new modeZero(),
		new modeOne(),
		new modeTwo(),
		new modeThree(),
		null, // no mode four :(.
		new modeFive(),
		null, // no mode six :(
		null, // no mode seven :(
	};

	public AutoModes() { }

	/**
	 * Sets the current autonomous mode based on the value of the
	 * driver station's binary switches.
	 */
    public void setAutoMode() {
		// Must reset to zero in case this method is
		// called more than once.
		selectedMode = 0;

		// gets numbers from the three auto switches, 
		// set selected mode to their sum in binary
		selectedMode += MechanismsJoystick.autoSwitchOne() ? 1 : 0;
       	selectedMode += MechanismsJoystick.autoSwitchTwo() ? 2 : 0;
		selectedMode += MechanismsJoystick.autoSwitchThree() ? 4 : 0;
        SmartDashboard.putNumber("Selected Auto", selectedMode);
    } 

	/**
	 * Gets the current status of the autonomous mode binary
	 * switches then uses an autonomous mode based on their value.
	 *
	 * The autonomous mode cannot be switched after calling this
	 * method. To do so you would need to create a new instance
	 * of this class.
	 */
    public void runAuto() {
		// get the most up to date status of the auto mode binary switches.
		setAutoMode();

		// prevent this method from producing too many threads
		// in case it is run in a loop, which it shouldn't, but still.
		if (autoThread != null)
			return;

		// set mode
		try {
			autoThread = availableModes[selectedMode];
		} catch (NullPointerException e) {
			RobotMap.dashboardLog.logError("You selected mode " + selectedMode + " which is not yet implemented.");
			RobotMap.dashboardLog.logError(e);
			return;
		}

		autoThread.start();
    }
}

