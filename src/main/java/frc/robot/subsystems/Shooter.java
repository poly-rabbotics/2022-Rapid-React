package frc.robot.subsystems;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.Controls.MechanismsJoystick;

public class Shooter {
    
    static double shooterSpeedSetpoint;
    static CANSparkMax shooterMotor;
    static SparkMaxPIDController shooterPIDController;
    static double kP, kI, kD;
    public Shooter() {
        shooterSpeedSetpoint = 0;
        //shooterMotor = RobotMap.shooterMotor;
        kP = 0.0001;
        kI = 0.000001;
        kD = 0.0001;
        shooterPIDController =  shooterMotor.getPIDController();
        shooterPIDController.setP(kP);
        shooterPIDController.setI(kI);
        shooterPIDController.setD(kD);

    }
    public void run() {
        if (MechanismsJoystick.shooterSpeedIncrease()) {
            SmartDashboard.putBoolean("speed increase", true);
            shooterSpeedSetpoint += 500;
        } else SmartDashboard.putBoolean("speed increase", false);

        if (MechanismsJoystick.shooterSpeedDecrease()) {
            SmartDashboard.putBoolean("speed decrease", true);
            shooterSpeedSetpoint -= 500;
        } else SmartDashboard.putBoolean("speed decrease", false);

        if (MechanismsJoystick.shooterButton()) {
            shooterPIDController.setReference(shooterSpeedSetpoint, ControlType.kVelocity);
        } else shooterPIDController.setReference(0, ControlType.kVelocity);

        SmartDashboard.putNumber("Shooter speed setpoint", shooterSpeedSetpoint);
        SmartDashboard.putNumber("Shooter RPM", shooterMotor.getEncoder().getVelocity());
    }
    
}
