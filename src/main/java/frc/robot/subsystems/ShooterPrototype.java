package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Controls.MechanismsJoystick;

public class ShooterPrototype {
    static double shooterSpeed = 0;
    static final CANSparkMax shooterMotor = new CANSparkMax(0, MotorType.kBrushless);

    public static void run() {
        if (MechanismsJoystick.shooterSpeedIncrease()) {
            SmartDashboard.putBoolean("speed increase", true);
            shooterSpeed += 0.05;
        } else             SmartDashboard.putBoolean("speed increase", false);


        if (MechanismsJoystick.shooterSpeedDecrease()) {
            SmartDashboard.putBoolean("speed decrease", true);
            shooterSpeed -= 0.05;
        } else             SmartDashboard.putBoolean("speed decrease", false);


       

        if (MechanismsJoystick.shooterButton()) {
            shooterMotor.set(shooterSpeed);
        }
        SmartDashboard.putNumber("shooter speed", shooterSpeed);

    }
}
