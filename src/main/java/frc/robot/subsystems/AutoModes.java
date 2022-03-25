package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Controls.MechanismsJoystick;

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

    public static void autoModeZero() { //DO NOTHING
        autoShooter.autoRun(0,15,0);
        autoConveyor.autoRun(0, 15, 0);
        autoIntake.autoRun(0, 15, 0);
    }
    
    public static void autoModeOne() { //TWO BALL AUTO, needs testing
        /*
        RobotMap.drivePancake.set(Value.kForward);
        autoShooter.autoRun(0, 15, -4600);
        autoConveyor.autoRun(1, 4, 0.7);
        autoConveyor.autoRun(4, 6, 0);
        //conveyor.autoRun(4, 5, 0);
        autoIntake.deployIntake(2, 5, true);
        autoDrive.moveByInches(3, 5, 60); 
        autoIntake.autoRun(3, 15, 0.8);
        autoDrive.moveByInches(6, 8, 0);
        autoConveyor.autoRun(6, 15, 0.7);
        */
        RobotMap.drivePancake.set(Value.kForward);
        autoIntake.deployIntake(0, 2, true);
        autoDrive.moveByInches(0, 5, 50);
        autoIntake.autoRun(3, 6, .8);
        autoIntake.autoRun(6, 15, 0);
        autoConveyor.autoRun(5, 6, 0.3);
        autoConveyor.autoRun(6, 10, 0);
        autoDrive.moveByInches(6, 9, -90);
        autoDrive.turnByDegrees(9, 11, Limelight.getX());
        autoShooter.autoRun(9, 15, -4600);
        autoConveyor.autoRun(11, 15, 0.7);
    }

    public static void autoModeTwo() { //shoot 1 ball and leave tarmac
        autoShooter.autoRun(0, 6, -4600);
        autoConveyor.autoRun(1, 6, 0.7);
        autoIntake.deployIntake(2, 5, true);
        autoDrive.moveByInches(3, 5, 60);
    }

    public static void autoModeThree() { //3 ball auto from right position
        
    }

    public static void autoModeFour() { //3 ball auto from left/mid position

    }

    public static void autoModeFive() { //failed four ball auto (fix it!)
        autoDrive.moveByInches(0, 1, 64);
        autoIntake.deployIntake(1, 4, true);
        autoIntake.autoRun(1, 4, 0.4);
        autoDrive.moveByInches(4, 6, -118);

        autoConveyor.autoRun(4, 8, 0.7);
        autoIntake.deployIntake(5, 7, false);
        autoShooter.autoRun(4, 7, -4600);
        autoDrive.moveByInches(7, 9, 277);
        
        autoIntake.deployIntake(9, 11, true);
        autoIntake.autoRun(9, 11, 0.4);
        autoIntake.deployIntake(11, 13, false);

        autoDrive.moveByInches(11, 13, 277);
        autoConveyor.autoRun(11, 13, 0.7);
        autoShooter.autoRun(13, 15, -4600);
    }

    public static void autoModeSix() { 

    }

    public static void autoModeSeven() {
        
    }

}
