package frc.robot.subsystems;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.Robot;
import frc.robot.Controls.MechanismsJoystick;

public class Shooter {
    
    static double shooterSpeedSetpoint;
    static CANSparkMax shooterMotor;
    static SparkMaxPIDController shooterPIDController;
    static double kP, kI, kD;
    public static LEDLights LEDLights;
    
    public Shooter() {

        LEDLights = new LEDLights();
        shooterSpeedSetpoint = 0;
        shooterMotor = RobotMap.shooterMotor;
        kP = 0.0001;
        kI = 0.000001;
        kD = 0.0001;
        shooterPIDController =  shooterMotor.getPIDController();
        shooterPIDController.setP(kP);
        shooterPIDController.setI(kI);
        shooterPIDController.setD(kD);

    }
    public void run() {
       if(MechanismsJoystick.shooterActive()) {
           shooterSpeedSetpoint=-7000;
           shooterPIDController.setReference(-7000, ControlType.kVelocity);
           LEDLights.up(2);
              }
       else {
           shooterSpeedSetpoint=0;
           shooterPIDController.setReference(0, ControlType.kVelocity);
       }
        SmartDashboard.putNumber("Shooter speed setpoint", shooterSpeedSetpoint);
        SmartDashboard.putNumber("Shooter RPM", shooterMotor.getEncoder().getVelocity());
    }
    
    public static void autoRun(double startTime, double endTime, double shooterSpeed) {
        double time = Robot.timer.get();
        if (time > startTime && time < endTime) {
          shooterPIDController.setReference(shooterSpeed, ControlType.kVelocity);
        } else shooterPIDController.setReference(0, ControlType.kVelocity);
    }
}
