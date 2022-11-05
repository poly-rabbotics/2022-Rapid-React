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
    
    static double shooterSpeedSetpoint, HIGH_SPEED_SETPOINT, LOW_SPEED_SETPOINT, AUTONOMOUS_SHOOTER_SETPOINT;
    static double HIGH_SPEED_SETPOINT_PID, LOW_SPEED_SETPOINT_PID;
    static double ENCODER_COUNTS_PER_REVOLUTION, ENC_PER_MS_TO_RPM, RPM_TO_ENC_PER_MS;
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
        HIGH_SPEED_SETPOINT_PID = -4630; //rpm
        LOW_SPEED_SETPOINT_PID = -2500; //rpm
        
        HIGH_SPEED_SETPOINT = -0.82;
        LOW_SPEED_SETPOINT = -0.35;
        shooterMotor = RobotMap.shooterMotor;
        
        ENCODER_COUNTS_PER_REVOLUTION = 2928;
        ENC_PER_MS_TO_RPM = 0.20492; //multiply native velocity units by this to get RPM
        RPM_TO_ENC_PER_MS = 4.88;
        kP = 0.14; //NEW PIDS NEEDED FOR FALCON
        kI = 0.0000;
        kD = 0.1;

        AUTONOMOUS_SHOOTER_SETPOINT = HIGH_SPEED_SETPOINT_PID * RPM_TO_ENC_PER_MS;
        //shooterPIDController =  shooterMotor.getPIDController();

        //typical falcon config, copied and adapted from drive
        shooterMotor.configFactoryDefault();
        shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
        shooterMotor.configNominalOutputForward(0);
        shooterMotor.configNominalOutputReverse(0);
        shooterMotor.configPeakOutputForward(1);
        shooterMotor.configPeakOutputReverse(-1);
        shooterMotor.setSensorPhase(false);
        
        shooterMotor.config_kP(0, kP);
        shooterMotor.config_kI(0, kI);
        shooterMotor.config_kD(0, kD);
        shooterMotor.config_kF(0, 0);
        
        shooterMotor.selectProfileSlot(0, 0);
        shooterMotor.setSelectedSensorPosition(0);
        
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
        
        shooterSpeedSetpoint = HIGH_SPEED_SETPOINT_PID * RPM_TO_ENC_PER_MS;
        //shooterSpeedSetpoint = highHubSetpointPID;
        //if(conveyorDelay.get()>1.5) Conveyor.conveyorSpeed=0.8;
        
        } else if (MechanismsJoystick.closeShot()) {
            shooterSpeedSetpoint = LOW_SPEED_SETPOINT_PID * RPM_TO_ENC_PER_MS;
            //shooterSpeedSetpoint = lowHubSetpointPID;
        }
         else {
           shooterSpeedSetpoint = 0;
       } 

       //shooterPIDController.setReference(shooterSpeedSetpoint, ControlType.kVelocity); THIS BUT FALCON INSTEAD
       shooterMotor.set(ControlMode.Velocity, shooterSpeedSetpoint);

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
        SmartDashboard.putNumber("Shooter speed setpoint", HIGH_SPEED_SETPOINT_PID);
        SmartDashboard.putNumber("Shooter RPM", shooterMotor.getSelectedSensorVelocity() * ENC_PER_MS_TO_RPM);
        SmartDashboard.putNumber("Shooter Encoder Counts", shooterMotor.getSelectedSensorPosition());

    }
    
    public void autoRun(double startTime, double endTime, double shooterSpeed) {
        double time = Robot.autoTimer.get();
        if (time > startTime && time < endTime) {
          shooterMotor.set(ControlMode.Velocity, shooterSpeed);
        }
    }

    public void adjustShooterSpeed() {
        if (MechanismsJoystick.axis1() > 0.5) {
            HIGH_SPEED_SETPOINT_PID += 2;
        } else if (MechanismsJoystick.axis1() < -0.5) {
            HIGH_SPEED_SETPOINT_PID -= 2;
        }
    }
}
