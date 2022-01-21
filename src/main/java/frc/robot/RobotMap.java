package frc.robot;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
public class RobotMap {
    
    public static final Joystick driveJoystick = new Joystick(0);
    public static final Joystick mechanismsJoystick = new Joystick(1);
    public static final Joystick configJoystick = new Joystick(2);

    public static final TalonSRX leftBack = new TalonSRX(1);
    public static final TalonSRX leftFront = new TalonSRX(2);
    public static final TalonSRX rightBack = new TalonSRX(3);
    public static final TalonSRX rightFront = new TalonSRX(4);

    
    public static final CANSparkMax intake = new CANSparkMax(6, MotorType.kBrushless);

    public static final CANSparkMax shooterMotor = new CANSparkMax(0, MotorType.kBrushless);

    public static final DoubleSolenoid intakePiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
}
