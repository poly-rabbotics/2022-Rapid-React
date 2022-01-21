package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.Controls.MechanismsJoystick;

public class ShooterPrototype {
    static double shooterSpeed = 0;
    static  CANSparkMax shooterMotor = RobotMap.shooterMotor;

    public static void run() {
        if (MechanismsJoystick.shooterPSpeedIncrease()) {
            SmartDashboard.putBoolean("speed increase", true);
            shooterSpeed += 0.05;
        } else SmartDashboard.putBoolean("speed increase", false);


        if (MechanismsJoystick.shooterPSpeedDecrease()) {
            SmartDashboard.putBoolean("speed decrease", true);
            shooterSpeed -= 0.05;
        } else SmartDashboard.putBoolean("speed decrease", false);


       

        if (MechanismsJoystick.shooterPButton()) {
            shooterMotor.set(shooterSpeed);
        } else shooterMotor.set(0);

        SmartDashboard.putNumber("Shooter speed setpoint", shooterSpeed);
        SmartDashboard.putNumber("Shooter RPM", shooterMotor.getEncoder().getVelocity());
    }
}
