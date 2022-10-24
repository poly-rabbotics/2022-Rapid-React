package frc.robot.subsystems;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.Robot;
import frc.robot.Controls.MechanismsJoystick;

public class Shooter {
    
    static double shooterSpeedSetpoint, HIGH_SPEED_SETPOINT, LOW_SPEED_SETPOINT;
    //static CANSparkMax shooterMotor;
    static TalonSRX shooterMotor;
    //static SparkMaxPIDController shooterPIDController;
    static double kP, kI, kD;
    public static LEDLights LEDLights;
    public boolean shooterRunning;
    public boolean shooterUpToSpeed;
    Timer conveyorDelay = new Timer();

    
    public Shooter() {
        LEDLights = new LEDLights();
        shooterSpeedSetpoint = 0;
        //highHubSetpoint = -4650;
        //lowHubSetpoint = -2500;
        HIGH_SPEED_SETPOINT = -0.82;
        LOW_SPEED_SETPOINT = -0.25;
        shooterMotor = RobotMap.shooterMotor;
        
        kP = 0.001; //NEW PIDS NEEDED FOR FALCON
        kI = 0.000000;
        kD = 0.000000;
        //shooterPIDController =  shooterMotor.getPIDController();

        //typical falcon config, copied and adapted from drive
        shooterMotor.configFactoryDefault();
        shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
        shooterMotor.configNominalOutputForward(0);
        shooterMotor.configNominalOutputReverse(0);
        shooterMotor.configPeakOutputForward(1);
        shooterMotor.configPeakOutputReverse(-1);
        shooterMotor.setSensorPhase(false);
        /*
        shooterMotor.config_kP(0, kP);
        shooterMotor.config_kI(0, kI);
        shooterMotor.config_kD(0, kD);
        shooterMotor.config_kF(0, 0);
        */
        shooterMotor.selectProfileSlot(0, 0);
        


        


        shooterRunning = false;
    }
    public void run() {
        if (MechanismsJoystick.arm()) {
            adjustShooterSpeed();
        }

        if(MechanismsJoystick.farShotPressed())
        {
            conveyorDelay.reset();
            conveyorDelay.start();
        }
        
       if (MechanismsJoystick.farShot()) {
        
        shooterSpeedSetpoint = HIGH_SPEED_SETPOINT;
        //if(conveyorDelay.get()>1.5) Conveyor.conveyorSpeed=0.8;
        LEDLights.up(2);
        } else if (MechanismsJoystick.closeShot()) {
            shooterSpeedSetpoint = LOW_SPEED_SETPOINT;
        }
         else {
           shooterSpeedSetpoint = 0;
       } 

       //shooterPIDController.setReference(shooterSpeedSetpoint, ControlType.kVelocity); THIS BUT FALCON INSTEAD
       shooterMotor.set(ControlMode.PercentOutput, shooterSpeedSetpoint);

       shooterRunning = shooterMotor.getSelectedSensorVelocity() != 0;
       shooterUpToSpeed = shooterMotor.getSelectedSensorVelocity() < -3000; 
       SmartDashboard.putBoolean("shooter up to speed?", shooterUpToSpeed);
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
        SmartDashboard.putNumber("Shooter speed setpoint", HIGH_SPEED_SETPOINT);
        SmartDashboard.putNumber("Shooter RPM", shooterMotor.getSelectedSensorVelocity());
    }
    
    public void autoRun(double startTime, double endTime, double shooterSpeed) {
        double time = Robot.autoTimer.get();
        if (time > startTime && time < endTime) {
          shooterMotor.set(ControlMode.Velocity, shooterSpeed);
        }
    }

    public void adjustShooterSpeed() {
        if (MechanismsJoystick.axis1() > 0.5) {
            HIGH_SPEED_SETPOINT += 2;
        } else if (MechanismsJoystick.axis1() < -0.5) {
            HIGH_SPEED_SETPOINT -= 2;
        }
    }
}
