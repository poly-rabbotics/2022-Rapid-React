package frc.robot;

import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AnalogInput;

public class RobotMap {
    
    public static XboxController driveJoystick;
    public static Joystick mechanismsJoystick;
    public static Joystick climbJoystick;
    public static Joystick guitarJoystick;
    
    public static void initJoysticks(){
        /*
        driveJoystick = new XboxController(0);
        mechanismsJoystick = new Joystick(1);
        climbJoystick = new Joystick(2);
        guitarJoystick = new Joystick(3); 
        */
        try{
            driveJoystick = new XboxController(0);
            mechanismsJoystick = new Joystick(1);
            climbJoystick = new Joystick(2);
            guitarJoystick = new Joystick(3); 

        } catch (Exception e) {
            SmartDashboard.putString("Error Readouts", "Error in RobotMap: Joysticks");
            SmartDashboard.putString("Error Readouts Detailed", e.getMessage());
        } 
    }
     
    public static TalonSRX leftBack;
    public static TalonSRX leftFront;
    public static TalonSRX rightBack;
    public static TalonSRX rightFront;
    public static TalonSRX staticArmWinch;
    public static TalonSRX dynamicArmWinch;

    public static CANSparkMax intakeMotor;
    public static CANSparkMax conveyorMotor;
    //public static CANSparkMax shooterMotor;
    public static TalonSRX shooterMotor;

    public static DoubleSolenoid drivePancake;
    public static DoubleSolenoid staticArmPancake;
    public static DoubleSolenoid dynamicArmPancake;
    public static DoubleSolenoid dynamicArmPivot;
    public static DoubleSolenoid intakeSolenoid;
    public static final AddressableLED led = new AddressableLED(1);

    public static void initDriveMotors() {
        try{
            leftBack = new TalonSRX(1);
            leftFront = new TalonSRX(2); 
            rightBack = new TalonSRX(3);
            rightFront = new TalonSRX(4);
        } catch (Exception e) {
            SmartDashboard.putString("Error Readouts", "Error in RobotMap: Drive Motors");
            SmartDashboard.putString("Error Readouts Detailed", e.getMessage());
        }
    }
    public static void initDrivePancakes() {
        try {
        drivePancake = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
        } catch (Exception e) {
            SmartDashboard.putString("Error Readouts", "Error in RobotMap: Drive Pancakes");
            SmartDashboard.putString("Error Readouts Detailed", e.getMessage());
        }
    }
    
    public static void initIntake() {
        try {
            intakeMotor = new CANSparkMax(5, MotorType.kBrushless);
            intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);
        } catch (Exception e) {
            SmartDashboard.putString("Error Readouts", "Error in RobotMap: Intake");
            SmartDashboard.putString("Error Readouts Detailed", e.getMessage());
        }
    }

    public static void initConveyor() {
        try {
            conveyorMotor = new CANSparkMax(6, MotorType.kBrushless);
        } catch (Exception e) {
            SmartDashboard.putString("Error Readouts", "Error in RobotMap: Conveyor");
            SmartDashboard.putString("Error Readouts Detailed", e.getMessage());
        }

    }

    public static void initShooter() {
        try {
            //shooterMotor = new CANSparkMax(7, MotorType.kBrushless);
            shooterMotor = new TalonSRX(7);
        } catch (Exception e) {
            SmartDashboard.putString("Error Readouts", "Error in RobotMap: Shooter");
            SmartDashboard.putString("Error Readouts Detailed", e.getMessage());
        }
    }
    
    public static void initClimb() {
        try {
            staticArmWinch = new TalonSRX(5);
            dynamicArmWinch = new TalonSRX(6);
            dynamicArmPivot = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);
            dynamicArmPancake = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);
            staticArmPancake = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8, 9);
        } catch (Exception e) {
            SmartDashboard.putString("Error Readouts", "Error in RobotMap: Climb");
            SmartDashboard.putString("Error Readouts Detailed", e.getMessage());
        }
    }
    
    //public static final DoubleSolenoid testSolenoidOne = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
    //public static final DoubleSolenoid testSolenoidTwo = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);
    //public static final DoubleSolenoid testSolenoidThree = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);
    //I MADE THIS FOER THE PROGRAMMING MEETING
    public static DigitalInput limitSwitchDA;
    public static DigitalInput limitSwitchSA;
    
    public static void initLimitSwitches() {
        try {
            limitSwitchDA = new DigitalInput(0);
            limitSwitchSA = new DigitalInput(1);

        }  catch (Exception e) {
            SmartDashboard.putString("Error Readouts", "Error in RobotMap: Limit Switches");
            SmartDashboard.putString("Error Readouts Detailed", e.getMessage());
        }
    }

    public static DigitalInput proxSensorLow; 
    public static DigitalInput proxSensorHigh;

    public static void initProxSensors() {
        try {
            proxSensorLow = new DigitalInput(2);
            proxSensorHigh = new DigitalInput(3);
        }  catch (Exception e) {
            SmartDashboard.putString("Error Readouts", "Error in RobotMap: Prox Sensors");
            SmartDashboard.putString("Error Readouts Detailed", e.getMessage());
        }
    }

    public static Servo limelightServo;
    public static Limelight limelight;
	public static ScheduledExecutorService limelightService;

    public static void initLimelight() {
        try {
            limelightService = Executors.newSingleThreadScheduledExecutor();
            limelightServo = new Servo(0);
            limelight = new Limelight();
        }  catch (Exception e) {
            SmartDashboard.putString("Error Readouts", "Error in RobotMap: Limelight");
            SmartDashboard.putString("Error Readouts Detailed", e.getMessage());
        }
    }

    public static AnalogInput pressureTransducer;

    public static void initPressureTransducer() {
        try {
            pressureTransducer = new AnalogInput(3);
        }  catch (Exception e) {
            SmartDashboard.putString("Error Readouts", "Error in RobotMap: Pressure Transducer");
            SmartDashboard.putString("Error Readouts Detailed", e.getMessage());
        }
    }

}
