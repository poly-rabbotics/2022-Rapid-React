package frc.robot;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
public class RobotMap {
    
    public static final Joystick driveJoystick = new Joystick(0);
    public static final Joystick mechanismsJoystick = new Joystick(1);
    public static final Joystick configJoystick = new Joystick(2);
    
    public static TalonSRX leftBack;
    public static TalonSRX leftFront;
    public static TalonSRX rightBack;
    public static TalonSRX rightFront;
    public static TalonSRX staticArmWinch;
    public static TalonSRX dynamicArmWinch;

    public static CANSparkMax intakeMotor;
    public static CANSparkMax conveyorMotor;
    public static CANSparkMax shooterMotor;

    public static DoubleSolenoid drivePancake;
    public static DoubleSolenoid staticArmPancake;
    public static DoubleSolenoid dynamicArmPancake;
    public static DoubleSolenoid dynamicArmPivot;
    public static DoubleSolenoid intakeSolenoid;

    public static void initDriveMotors() {
        leftBack = new TalonSRX(1);
        leftFront = new TalonSRX(2); 
        rightBack = new TalonSRX(3);
        rightFront = new TalonSRX(4); 
    }
    public static void initDrivePancakes() {
        drivePancake = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
    }
    
    public static void initIntake() {
        intakeMotor = new CANSparkMax(5, MotorType.kBrushless);
        //intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);   
    }

    public static void initConveyor() {
        conveyorMotor = new CANSparkMax(7, MotorType.kBrushless);

    }

    public static void initShooter() {
        shooterMotor = new CANSparkMax(6, MotorType.kBrushless);
    }
    
    public static void initClimb() {
        staticArmWinch = new TalonSRX(8);
        dynamicArmWinch = new TalonSRX(9);
        staticArmPancake = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);
        dynamicArmPancake = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);
        dynamicArmPivot = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8, 9);
    }
    
    //public static final DoubleSolenoid testSolenoidOne = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
    //public static final DoubleSolenoid testSolenoidTwo = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);
    //public static final DoubleSolenoid testSolenoidThree = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);

    public static final DigitalInput magLimitSwitch = new DigitalInput(0);
    public static final Servo limelightServo = new Servo(0);

}
