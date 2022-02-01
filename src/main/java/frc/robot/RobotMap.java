package frc.robot;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
public class RobotMap {
    
    public static final Joystick driveJoystick = new Joystick(0);
    public static final Joystick mechanismsJoystick = new Joystick(1);
    public static final Joystick configJoystick = new Joystick(2);
    
    public static TalonSRX leftBack;
    public static TalonSRX leftFront;
    public static TalonSRX rightBack;
    public static TalonSRX rightFront;

    public static CANSparkMax intakeMotor;

    public static DoubleSolenoid drivePancake;

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
    }
    
    
    //public static final DoubleSolenoid testSolenoidTwo = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);



    

    //public static final CANSparkMax shooterMotor = new CANSparkMax(6, MotorType.kBrushless); //NEO

    //public static final DoubleSolenoid intakePiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
}
