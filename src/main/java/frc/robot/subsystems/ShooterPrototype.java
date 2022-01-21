package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Controls.MechanismsJoystick;

public class ShooterPrototype {
    static double shooterSpeed;

    public static CANSparkMax shooterMotor;
    public static SparkMaxPIDController shooterPIDController;
    static double kP, kI, kD;
    public ShooterPrototype() {
        shooterSpeed = 0;
        shooterMotor = RobotMap.shooterMotor;
        shooterPIDController = shooterMotor.getPIDController();
        kP = 0.0;
        kI = 0.0;
        kD = 0.0;
    }
    public static void run() {
        if (MechanismsJoystick.shooterSpeedIncrease()) {
            SmartDashboard.putBoolean("speed increase", true);
            shooterSpeed += 0.05;
        } else SmartDashboard.putBoolean("speed increase", false);


        if (MechanismsJoystick.shooterSpeedDecrease()) {
            SmartDashboard.putBoolean("speed decrease", true);
            shooterSpeed -= 0.05;
        } else SmartDashboard.putBoolean("speed decrease", false);


       

        if (MechanismsJoystick.shooterButton()) {
            shooterMotor.set(shooterSpeed);
        } else shooterMotor.set(0);

        SmartDashboard.putNumber("shooter speed", shooterSpeed);

    }
}
