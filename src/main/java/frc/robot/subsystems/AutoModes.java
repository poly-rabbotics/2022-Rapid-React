package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Controls.MechanismsJoystick;
import frc.robot.subsystems.autoModes.*;

public class AutoModes {
   	private Thread autoThread; 
	private static int selectedMode = 0;

	public AutoModes() { }

    public void setAutoMode() {
		// gets numbers from the three auto switches, 
		// set selected mode to their sum in binary
		selectedMode += MechanismsJoystick.autoSwitchOne() ? 1 : 0;
       	selectedMode += MechanismsJoystick.autoSwitchTwo() ? 2 : 0;
		selectedMode += MechanismsJoystick.autoSwitchThree() ? 4 : 0;
        SmartDashboard.putNumber("Selected Auto", selectedMode);
    } 

    public void runAuto() {
        switch (selectedMode) {
            case 0: 
                autoThread = new modeZero();
				break;
            case 1: 
                autoThread = new modeOne();
				break;
            case 2:
                autoThread = new modeTwo();
                break;
            case 3:
                autoModeThree();
                break;
            case 4: 
                autoModeFour(); 
                break;  
            case 5: 
                autoModeFive(); 
                break;
            case 6: 
                autoModeSix(); 
                break;
            case 7: 
                autoModeSeven(); 
                break;
            default: 
                break;
        }
    }

	/** 
	 * Start Autonomous thread.
	 */
    public void runAutoThread() { 
		// prevent this method from producing too many threads
		// in case it is run in a loop, which it shouldn't, but still.
		if (autoThread != null)
			return;

		autoThread = new modeZero();
		autoThread.start();
	}

    public static void autoModeThree() { //3 ball auto from right position WORKING
        //USE THIS
        autoIntake.deployIntake(0, 1, true);
        autoDrive.resetEncodersCall(0, 0.05);
        autoIntake.autoRun(0, 15, -0.85);
        autoShooter.autoRun(0, 3, -4800);
        autoConveyor.autoRun(2, 3, 1);
        autoShooter.autoRun(3, 3.1, 0);
        autoConveyor.autoRun(3, 3.1, 0);
        autoDrive.goToEncCountsTurn(0 +1.5, 1+1.5, 6000);
        autoDrive.resetEncodersCall(1+1.5, 1.01+1.5);
        autoDrive.goToEncCounts(1.01+1.5,3+1.5,90000);
        autoDrive.resetEncodersCall(3+1.5, 3.01+1.5);
        autoConveyor.autoRun(3+1.5, 5+1.5, 0.3);
        autoConveyor.autoRun(5+1.5, 5.1+1.5, 0);
        autoDrive.goToEncCounts(3.01+1.5, 4.5+1.5, -80000);
        autoDrive.resetEncodersCall(4.5+1.5, 4.51+1.5);
        autoDrive.goToEncCountsTurn(4.51+1.5, 5.5+1.5, -25000);
        autoDrive.resetEncodersCall(5.5+1.5, 5.51+1.5);
        autoDrive.goToEncCounts(5.51+1.5, 7.5+1.5, 130000);
        autoDrive.resetEncodersCall(7.5+1.5, 7.51+1.5);
        autoDrive.goToEncCounts(7.51+1.5, 9.5+1.5, -120000);
        autoDrive.resetEncodersCall(9.5+1.5, 9.51+1.5);
        autoDrive.goToEncCountsTurn(9.51+1.5 ,10.5+1.5, 20000);
        autoShooter.autoRun(10.5+1.5, 15+1.5, -4700);
        autoConveyor.autoRun(12.2+1.5, 15+1.5, 0.8);
        autoDrive.resetEncodersCall(10.5+1.5, 10.51+1.5);
        autoDrive.goToEncCounts(10.51+1.5, 12.5+1.5, -30000);

    }

    public static void autoModeFour() { 
       
    }

    public static void autoModeFive() { //failed four ball auto (NEEDS TESTING BEFORE RUNNING)

        //DO NOT USE!!!
        autoIntake.deployIntake(0, 1, true);
        autoDrive.moveByInches(0, 1.5, 21);
        autoDrive.turnByDegrees(1.5, 3, 47);
        autoDrive.moveByInches(3, 4.5, 54);
        autoIntake.autoRun(4, 6, -0.8);
        autoIntake.autoRun(6, 17, 0);
        autoConveyor.autoRun(5, 6, 0.4);
        autoConveyor.autoRun(6, 7, 0);
        autoDrive.turnByDegrees(6, 7, -20);
        autoDrive.moveByInches(7, 8.5, -103);
        autoDrive.turnByDegrees(8.5, 9.5, -51);
        autoDrive.moveByInches(9.5, 10.5, -23);
        autoShooter.autoRun(9.5, 11, -4600);
        autoConveyor.autoRun(10.5, 12.5, 0.8);
        autoDrive.moveByInches(12.5, 13, 23);
        autoDrive.turnByDegrees(13, 14, 60);
        autoDrive.moveByInches(14, 16, 223);
        autoDrive.turnByDegrees(16, 17, -20);
        autoDrive.moveByInches(17, 18, 41);
        autoIntake.autoRun(17, 20, -0.8);
        autoConveyor.autoRun(18, 21, 0.3);
        autoConveyor.autoRun(21, 22, 0);
        autoDrive.turnByDegrees(19, 20, 43);
        autoDrive.moveByInches(20, 21, -35);
        autoDrive.turnByDegrees(21, 22, -28);
        autoDrive.moveByInches(22, 24, -232);
        autoDrive.turnByDegrees(24, 25, -50);
        autoDrive.moveByInches(25, 26, -26);
        autoShooter.autoRun(25, 28, -4600);
        autoConveyor.autoRun(26, 28, 0.8);

    }

    public static void autoModeSix() { 
       
    }

    public static void autoModeSeven() {
        
    }

}
