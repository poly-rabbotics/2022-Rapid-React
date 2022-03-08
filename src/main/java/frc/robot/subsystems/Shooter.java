package frc.robot.subsystems;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;
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
    public boolean upToSpeed;
    Timer conveyorDelay = new Timer();

    
    public Shooter() {
        LEDLights = new LEDLights();
        shooterSpeedSetpoint = 0;
        shooterMotor = RobotMap.shooterMotor;
        kP = 0.00013;
        kI = 0.000001;
        kD = 0.0001;
        shooterPIDController =  shooterMotor.getPIDController();
        shooterPIDController.setP(kP);
        shooterPIDController.setI(kI);
        shooterPIDController.setD(kD);
        upToSpeed = false;
    }
    public void run() {

        if(MechanismsJoystick.farShotPressed())
        {
            conveyorDelay.reset();
            conveyorDelay.start();
        }
        
       if (MechanismsJoystick.farShot()) {
        
        shooterSpeedSetpoint=-4500;
        if(conveyorDelay.get()>1.5) Conveyor.conveyorSpeed=0.8;
        LEDLights.up(2);
        } else if (MechanismsJoystick.closeShot()) {
            shooterSpeedSetpoint=-2500;
        }
         else {
           shooterSpeedSetpoint=0;
       } 
       shooterPIDController.setReference(shooterSpeedSetpoint, ControlType.kVelocity);


       upToSpeed = shooterMotor.getEncoder().getVelocity() < -4300;
       SmartDashboard.putBoolean("shooter up to speed?", upToSpeed);
       /*
       if(MechanismsJoystick.shooterActive()) {
        shooterSpeedSetpoint=-5000;
        shooterMotor.set(-0.85);
        LEDLights.up(2);
           }
    else {
        shooterSpeedSetpoint=0;
        shooterMotor.set(0);
    } */
        SmartDashboard.putNumber("Shooter speed setpoint", shooterSpeedSetpoint);
        SmartDashboard.putNumber("Shooter RPM", shooterMotor.getEncoder().getVelocity());
    }
    
    public void autoRun(double startTime, double endTime, double shooterSpeed) {
        double time = Robot.autoTimer.get();
        if (time > startTime && time < endTime) {
          shooterPIDController.setReference(shooterSpeed, ControlType.kVelocity);
        }
    }
}
