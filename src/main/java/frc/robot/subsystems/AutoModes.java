package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
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

        //EG: These cannot be re-constructed, that need to reference existing Robot objects
        autoShooter = Robot.shooter;
        autoConveyor = Robot.conveyor;
        autoIntake = Robot.intake;
        autoDrive = Robot.drive;
    }

    public static void setAutoMode() {
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

    public static void runAuto() {
        //EG: Case structures dont use brackets for each case... this may not hurt anything but it's not standard
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
            case 4: {autoModeFour(); break;}  // EG: Fix the rest of these...
            case 5: {autoModeFive(); break;}
            case 6: {autoModeSix(); break;}
            case 7: {autoModeSeven(); break;}
            default: 
                break;
        }
    }

    public static void autoModeZero() { //DO NOTHING
        autoShooter.autoRun(0,15,0);
        autoConveyor.autoRun(0, 15, 0);
        autoIntake.autoRun(0, 15, 0);
    }
    
    public static void autoModeOne() { //TWO BALL AUTO
        RobotMap.drivePancake.set(Value.kForward);
        autoShooter.autoRun(0, 15, -4600);
        autoConveyor.autoRun(1, 4, 0.7);
        //conveyor.autoRun(4, 5, 0);
        autoIntake.deployIntake(2, 5, true);
        autoDrive.moveByInches(3, 5, 60); //fix this distance setpoint for actual field geometry
        autoIntake.autoRun(3, 15, 0.5);
        autoDrive.moveByInches(6, 8, 0);
        autoConveyor.autoRun(4, 10, 0.2);
        autoConveyor.autoRun(10, 15, 0.7);
        autoIntake.deployIntake(7, 12, false);
        autoIntake.autoRun(6, 15, 0);
    }

    public static void autoModeTwo() { //shoot 1 ball and leave tarmac
        
    }

    public static void autoModeThree() { //delay, then shoot 1 and leave tarmac
        
    }

    public static void autoModeFour() {
        
    }

    public static void autoModeFive() {
        
    }

    public static void autoModeSix() {
        
    }

    public static void autoModeSeven() {
        
    }

}
