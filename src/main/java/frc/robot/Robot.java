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
import frc.robot.Controls.MechanismsJoystick;
import frc.robot.subsystems.AHRSGyro;
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
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  public static Intake intake;
  public static Shooter shooter;
  public static Conveyor conveyor;
  public static Climb climb;
  public static Drive drive;
  public static LEDLights LEDLights;
  public static Limelight limelight;
  public static Timer timer;
  public static AHRSGyro gyro;
  public static double leftEncoderCounts, rightEncoderCounts;

  Compressor comp;
  PneumaticHub hub;
  
  @Override
  public void robotInit() {
    //comp = new Compressor(1, PneumaticsModuleType.REVPH);
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    RobotMap.initShooter();
    shooter = new Shooter();
    //RobotMap.initDriveMotors();
    RobotMap.initIntake();
    intake = new Intake();

    //RobotMap.initDriveMotors();
    //RobotMap.initDrivePancakes();
    //drive = new Drive();

    RobotMap.initClimb();
    climb = new Climb();
    
    LEDLights = new LEDLights();

    timer = new Timer();
    timer.start();

    AHRSGyro.reset();
    gyro = new AHRSGyro();

    RobotMap.initConveyor();
    conveyor = new Conveyor();

    limelight = new Limelight();
    
    //comp.enableDigital();
    
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
    //SmartDashboard.putNumber("PSI", comp.getPressure());
    SmartDashboard.putNumber("Timer", timer.get());
    SmartDashboard.putBoolean("Limit Switch", !RobotMap.magLimitSwitch.get());
    SmartDashboard.putNumber("Gyro Degrees", gyro.getDegrees());
    
    //leftEncoderCounts = Drive.leftBack.getSelectedSensorPosition();
    //rightEncoderCounts = -1 * Drive.rightBack.getSelectedSensorPosition();
    SmartDashboard.putNumber("left Encoder Feet", leftEncoderCounts / 13445);
    SmartDashboard.putNumber("right Encoder Feet", rightEncoderCounts / 13445);
    SmartDashboard.putNumber("left Encoder Counts", leftEncoderCounts);
    SmartDashboard.putNumber("right Encoder Counts", rightEncoderCounts);
    SmartDashboard.putNumber("left Encoder Degrees", leftEncoderCounts / Drive.encoderCountsPer360/360);
    SmartDashboard.putNumber("right Encoder Degrees", rightEncoderCounts / Drive.encoderCountsPer360/360);
    SmartDashboard.putBoolean("prox 1", !RobotMap.proxSensor1.get());
    SmartDashboard.putBoolean("prox 2", !RobotMap.proxSensor2.get());

    if(isDisabled()) LEDLights.rainbow();
    //LEDLights.singleColor(0, 255, 0);
    limelight.run();

    //CLIMB DATA
    SmartDashboard.putNumber("DA encoder counts", Climb.dynamicArmWinch.getSelectedSensorPosition());
    SmartDashboard.putNumber("SA encoder counts", Climb.staticArmWinch.getSelectedSensorPosition());
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
    AHRSGyro.reset();
    m_autoSelected = m_chooser.getSelected();
    timer.reset();
    drive.initAutoDrive();
    drive.resetEncoders();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
    drive.turnByDegrees(0, 5, 90);
    //drive.moveByInches(0, 5, -36);
  

    //drive.turnByDegrees(30);
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    } 

    LEDLights.up(2);
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    AHRSGyro.reset();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    shooter.run();
    conveyor.run();
    //intake.run();
    //climb.run();
    //drive.run();
    LEDLights.GreenGold();
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
