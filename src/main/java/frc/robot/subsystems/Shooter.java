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
    
    static double shooterSpeedSetpoint, highHubSetpoint, lowHubSetpoint;
    static CANSparkMax shooterMotor;
    static SparkMaxPIDController shooterPIDController;
    static double kP, kI, kD;
    public static LEDLights LEDLights;
    public boolean shooterRunning;
    public static boolean shooterUpToSpeed;
    Timer conveyorDelay = new Timer();

    
    public Shooter() {
        LEDLights = new LEDLights();
        shooterSpeedSetpoint = 0;
        highHubSetpoint = 4600;
        lowHubSetpoint = 2500;
        shooterMotor = RobotMap.shooterMotor;
        kP = 0.00013;
        kI = 0.000001;
        kD = 0.0001;
        shooterPIDController =  shooterMotor.getPIDController();
        shooterPIDController.setP(kP);
        shooterPIDController.setI(kI);
        shooterPIDController.setD(kD);
        shooterRunning = false;
    }
    public void run() {

        if(MechanismsJoystick.farShotPressed())
        {
            conveyorDelay.reset();
            conveyorDelay.start();
        }
        
       if (MechanismsJoystick.farShot()) {
        
        shooterSpeedSetpoint = highHubSetpoint;
        //if(conveyorDelay.get()>1.5) Conveyor.conveyorSpeed=0.8;
        LEDLights.up(2);
        } else if (MechanismsJoystick.closeShot()) {
            shooterSpeedSetpoint = lowHubSetpoint;
        }
         else {
           shooterSpeedSetpoint=0;
       } 
       shooterPIDController.setReference(shooterSpeedSetpoint, ControlType.kVelocity);


       shooterRunning = shooterMotor.getEncoder().getVelocity() < -1000;
       shooterUpToSpeed = shooterMotor.getEncoder().getVelocity() < -4400; 
       SmartDashboard.putBoolean("shooter up to speed?", shooterRunning);
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

    public void adjustShooterSpeed() {
        if (MechanismsJoystick.axis0() > 0) {
            highHubSetpoint += 10;
        } else if (MechanismsJoystick.axis0() < 0) {
            highHubSetpoint -= 10;
        }
    }
}
