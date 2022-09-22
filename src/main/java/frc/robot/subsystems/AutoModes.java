package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Controls.MechanismsJoystick;
import frc.robot.subsystems.autoModes.*;

public class AutoModes {
    static Shooter autoShooter;
    static Conveyor autoConveyor;
    static Intake autoIntake; 
    static Drive autoDrive; 
    static int selectedMode;
    static int autoSwitchOne, autoSwitchTwo, autoSwitchThree;
    public AutoModes() {

        autoShooter = Robot.shooter;
        autoConveyor = Robot.conveyor;
        autoIntake = Robot.intake;
        autoDrive = Robot.drive;
    }

    public void setAutoMode() {
        if (MechanismsJoystick.autoSwitchOne()) autoSwitchOne = 1; //0 or 1
        else autoSwitchOne = 0;
        if (MechanismsJoystick.autoSwitchTwo()) autoSwitchTwo = 2; //0 or 2
        else autoSwitchTwo = 0;
        if (MechanismsJoystick.autoSwitchThree()) autoSwitchThree = 4; // 0 or 4
        else autoSwitchThree = 0;
        //gets numbers from the three auto switches, set selected mode to their sum in binary
        selectedMode = autoSwitchOne + autoSwitchTwo + autoSwitchThree;
        SmartDashboard.putNumber("Selected Auto", selectedMode);
    } 

    public void runAuto() {
        
        switch (selectedMode) {
            case 0: 
                autoModeZero(); 
                break;
            case 1: 
                autoModeOne(); 
                break;
            case 2:
                autoModeTwo();
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

	private modeZero modeZeroThread; 

    public void autoModeZero() { //DO NOTHING
		modeZeroThread = new modeZero();
		modeZeroThread.start();
	}
    
    public static void autoModeOne() { //TWO BALL AUTO
        autoIntake.deployIntake(0, 1, true);
        autoIntake.autoRun(0, 10, -0.85);
        autoShooter.autoRun(0, 3, -4600);
        autoConveyor.autoRun(1.5, 3, 1);
        autoConveyor.autoRun(3, 3.1, 0);
        autoDrive.goToEncCounts(3, 5, 100000);
        autoDrive.resetEncodersCall(5, 5.1);
        autoDrive.goToEncCounts(5.1, 7, -100000);
        autoConveyor.autoRun(7, 9, 1);
        autoShooter.autoRun(5.5, 10, -4600);
    }

    public static void autoModeTwo() { //shoot 1 ball and leave tarmac
        autoIntake.deployIntake(0, 1, true);
        autoIntake.autoRun(0, 10, -0.85);
        autoShooter.autoRun(0, 3, -4600);
        autoConveyor.autoRun(1.5, 3, 1);
        autoConveyor.autoRun(3, 3.1, 0);
        autoDrive.goToEncCounts(3, 5, 100000);
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
