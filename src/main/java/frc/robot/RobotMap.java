package frc.robot;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
public class RobotMap {
    
    public static final Joystick driveJoystick = new Joystick(0);
    public static final Joystick mechanismsJoystick = new Joystick(1);
    public static final Joystick configJoystick = new Joystick(2);

    public static final Talon leftBack = new Talon(1);
    public static final Talon leftFront = new Talon(2);
    public static final Talon rightBack = new Talon(3);
    public static final Talon rightFront = new Talon(4);

    public static final CANSparkMax intake = new CANSparkMax(5, MotorType.kBrushless);
    public static final DoubleSolenoid intakePiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    //register my changes PLEASE
    
}
