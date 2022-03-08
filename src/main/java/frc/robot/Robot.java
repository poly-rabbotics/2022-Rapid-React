// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Controls.DriveJoystick;
import frc.robot.Controls.MechanismsJoystick;
import frc.robot.subsystems.AHRSGyro;
import frc.robot.subsystems.AutoModes;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LEDLights;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  public static Intake intake;
  public static Shooter shooter;
  public static Conveyor conveyor;
  public static Climb climb;
  public static Drive drive;
  public static LEDLights LEDs;
  public static Limelight limelight;
  public static Timer masterTimer, autoTimer;
  public static AHRSGyro gyro;
  public static double leftEncoderCounts, rightEncoderCounts;
  boolean pressureGood;
  boolean isGyroReset = false;
  Compressor comp;
  PneumaticHub hub;
  
  @Override
  public void robotInit() {
    comp = new Compressor(1, PneumaticsModuleType.REVPH);
    
    RobotMap.initShooter();
    shooter = new Shooter();
    RobotMap.initIntake();
    intake = new Intake();

    RobotMap.initDriveMotors();  
    RobotMap.initDrivePancakes();
    drive = new Drive();

    RobotMap.initClimb();
    climb = new Climb();
    
    LEDs = new LEDLights();  //EG: Class name same as variable name.  It technically works but will get confusing

    masterTimer = new Timer();
    masterTimer.start();
    autoTimer = new Timer();
    

    gyro = new AHRSGyro();

    RobotMap.initConveyor();
    conveyor = new Conveyor();

    limelight = new Limelight();
    
    comp.enableDigital();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Timer", masterTimer.get());
    SmartDashboard.putBoolean("DA Limit Switch", !RobotMap.limitSwitchDA.get());
    SmartDashboard.putBoolean("SA Limit Switch", !RobotMap.limitSwitchSA.get());

    SmartDashboard.putNumber("Gyro Degrees", gyro.getDegrees());
    
    leftEncoderCounts = Drive.leftBack.getSelectedSensorPosition();
    rightEncoderCounts = -1 * Drive.rightBack.getSelectedSensorPosition();
    SmartDashboard.putNumber("left Encoder Feet", leftEncoderCounts / 13445);
    SmartDashboard.putNumber("right Encoder Feet", rightEncoderCounts / 13445);
    SmartDashboard.putNumber("left Encoder Counts", leftEncoderCounts);
    SmartDashboard.putNumber("right Encoder Counts", rightEncoderCounts);
    SmartDashboard.putNumber("left Encoder Degrees", leftEncoderCounts / Drive.encoderCountsPer360/360);
    SmartDashboard.putNumber("right Encoder Degrees", rightEncoderCounts / Drive.encoderCountsPer360/360);
    SmartDashboard.putBoolean("prox 1", RobotMap.proxSensorLow.get());
    SmartDashboard.putBoolean("prox 2", RobotMap.proxSensorHigh.get());

    double robotPressure = 40.16 * (RobotMap.pressureTransducer.getVoltage() - 0.52);
  
    SmartDashboard.putNumber("Robot Pressure",robotPressure ); 
    pressureGood = robotPressure > 60;
    SmartDashboard.putBoolean("Pressure Good?", pressureGood);  
    
    //if(isDisabled()) LEDs.rainbow();
//EG: Let's not hardcode this here, lets do LEDLights.pattern=4; and then call LEDLights.run();

    //LEDLights.singleColor(0, 255, 0);
    limelight.run();

    //CLIMB DATA
    SmartDashboard.putNumber("DA encoder counts", Climb.dynamicArmWinch.getSelectedSensorPosition());
    SmartDashboard.putNumber("SA encoder counts", Climb.staticArmWinch.getSelectedSensorPosition());

    if (masterTimer.get() > 5 && !isGyroReset) {
      gyro.reset();
      isGyroReset = true;
    }
    
    if (masterTimer.get() > 40 && masterTimer.get() < 41) {
      DriveJoystick.rumble(0.5);
    } else {
      DriveJoystick.rumble(0);
    }

    AutoModes.setAutoMode();

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    autoTimer.start();    // EG: This is the same timer as the one which resets the gyro, lets use a different one
    drive.initAutoDrive();
    drive.resetEncoders(); 
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
    AutoModes.runAuto();
    
    /*
    RobotMap.drivePancake.set(Value.kForward);
    shooter.autoRun(0, 15, -4600);
    conveyor.autoRun(1, 4, 0.7);
    //conveyor.autoRun(4, 5, 0);
    intake.deployIntake(2, 5, true);
    drive.moveByInches(3, 5, 60); //fix this distance setpoint for actual field geometry
    intake.autoRun(3, 6, 0.5);
    drive.moveByInches(6, 8, -5);
    conveyor.autoRun(4, 10, 0.2);
    conveyor.autoRun(10, 15, 0.7);
    intake.deployIntake(7, 12, false);
    intake.autoRun(6, 15, 0);
    LEDs.up(2);
    
    drive.turnByDegrees(10, 12, 180);
    drive.moveByInches(12, 15, -60);
    */
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    gyro.reset();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    shooter.run();
    conveyor.run();
    intake.run();
    climb.run();
    drive.run();
    limelight.run();
    //LEDLights.GreenGold();
    //if(MechanismsJoystick.arm()) LEDLights.nice();

    /*
    
    if(MechanismsJoystick.dynamicArmPancakeRelease()) {
    
      RobotMap.testSolenoidOne.set(Value.kOff);
      RobotMap.testSolenoidTwo.set(Value.kOff);
      RobotMap.testSolenoidThree.set(Value.kReverse);

    } else if (MechanismsJoystick.staticArmPancakeRelease()) {
      
      RobotMap.testSolenoidOne.set(Value.kOff);
      RobotMap.testSolenoidTwo.set(Value.kOff);
      RobotMap.testSolenoidThree.set(Value.kForward);
    }
    */

    
    
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() { }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
