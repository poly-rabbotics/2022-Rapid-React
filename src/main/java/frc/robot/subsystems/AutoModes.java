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
        
        /*
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
        */
    }

    public static void autoModeTwo() { //shoot 1 ball and leave tarmac
        autoShooter.autoRun(0, 6, -4600);
        autoConveyor.autoRun(1, 6, 0.7);
        autoIntake.deployIntake(2, 5, true);
        autoDrive.moveByInches(3, 5, 80);
    }

    public static void autoModeThree() { //3 ball auto from right position
        
    }

    public static void autoModeFour() { //3 ball auto from left/mid position

    }

    public static void autoModeFive() { //failed four ball auto (NEEDS TESTING BEFORE RUNNING)
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
        RobotMap.drivePancake.set(Value.kForward);
        if (autoDrive.moveByInches(0, 5, 36)) {
            autoDrive.turnByDegreesBasic(3, 5, 180);
        }
        //autoDrive.turnByDegreesBasic(3, 5, 180);
        //autoDrive.moveByInches(5, 8, 36);
        //autoDrive.turnByDegrees(5, 8, 90);
        //autoDrive.turnByDegrees(10, 13, -90);
    }

    public static void autoModeSeven() {
        autoDrive.moveByInches(0, 3, 48);
    }

}
