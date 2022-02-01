
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Controls.DriveJoystick;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Drive { 
  //Timer timer;
  double time, oldTime, cP_LL, cD_LL, cI_LL, power_LL, accumError; //Limelight PID control vars
  double x, oldX;
  static double fts_to_RPM;
  static double leftRPM, rightRPM, leftEncoderCounts, rightEncoderCounts;
  static double cP, cD, cI, leftPower, rightPower; // drive speed PID control vars
  static double leftSpeedError, rightSpeedError, leftSpeedSetpoint, rightSpeedSetpoint;
  static double move, turn, left, right; // move and turn for arcade drive, left and right for tank
  static double maxFtPerSec;
  Joystick calibrateJoy;
  static TalonSRX leftBack, leftFront, rightBack, rightFront;
  public static String driveMode;
  boolean isAligned;
  static double targetVLeft;
  static double targetVRight;
  private Field2d field = new Field2d();


  public Drive() {
    calibrateJoy = new Joystick(2);
    //timer = new Timer();
    //timer.start();
    cP_LL = 0.0425; //old LL PIDs
    cD_LL = 0.0173;
    cI_LL = 0.0014;
    //leftSpeedSetpoint = 0;
    //rightSpeedSetpoint = 0;
    move = 0;
    turn = 0;
    oldTime = 0;
    accumError = 0;
    //time = timer.get();
    x = 0;
    driveMode = "";
    
    targetVLeft = 0;
    targetVRight = 0;
    
    leftBack = RobotMap.leftBack;
    leftFront = RobotMap.leftFront;
    rightBack = RobotMap.rightBack;
    rightFront = RobotMap.rightFront;
    
    leftFront.follow(leftBack);
    rightFront.follow(rightBack);
    leftFront.setInverted(InvertType.FollowMaster);
    rightFront.setInverted(InvertType.FollowMaster);

    fts_to_RPM = 409.3;
    maxFtPerSec = 60; //find this out through testing

    leftBack.configFactoryDefault();
    rightBack.configFactoryDefault();
    leftBack.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
    rightBack.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
    leftBack.configNominalOutputForward(0);
    rightBack.configNominalOutputForward(0);
    leftBack.configNominalOutputReverse(0);
    rightBack.configNominalOutputReverse(0);
    leftBack.configPeakOutputForward(1);
    rightBack.configPeakOutputForward(1);
    leftBack.configPeakOutputReverse(-1);
    rightBack.configPeakOutputReverse(-1);

    leftBack.setSensorPhase(false);
    rightBack.setSensorPhase(false);

    leftBack.config_kP(0, .1); //determine best P
    rightBack.config_kP(0, .1);
    leftBack.config_kI(0, 0.0005); //determine best I
    rightBack.config_kI(0, .0005);
    leftBack.config_kD(0, .0001); //determine best D
    rightBack.config_kD(0, .0001);
    leftBack.config_kF(0, 0);
    rightBack.config_kF(0, 0);

    leftBack.selectProfileSlot(0, 0);
    rightBack.selectProfileSlot(0, 0);
  } 
  public static boolean isAutoDrive = false;
  public boolean intakeforward = true;
  //static SpeedControllerGroup leftDrive = new SpeedControllerGroup(RobotMap.leftFront, RobotMap.leftBack);
  //static SpeedControllerGroup rightDrive = new SpeedControllerGroup(RobotMap.rightFront, RobotMap.rightBack);
  static boolean driveSelection = DriveJoystick.driveMode();
  //public static DifferentialDrive drive = new DifferentialDrive(leftDrive, rightDrive);

  public Field2d getField() {
		return field;
	}
  
  void joystickDrive() {
    move = DriveJoystick.getMove(); 
    turn = DriveJoystick.getTurn(); 
    left = -DriveJoystick.getMove();
  
    right = -DriveJoystick.axisFive();
    if (DriveJoystick.driveMode())
      driveSelection = !driveSelection;
    // driveSelection = DriveJoystick.driveMode();
    
    move = Math.signum(move) * Math.pow(move, 2);
    turn = Math.pow(turn, 3);

    
    if ((move<0.05) && (move>-0.05)) { //deadzone between -5% and 5%
      move = 0;
    }

    move();
    

    //leftBack.setReference((move * 12.21 * fts_to_RPM) + turn * 12.21 * fts_to_RPM, ControlMode.Velocity);
    //rightBack.setReference((-move * 12.21 * fts_to_RPM) - turn * 12.21 * fts_to_RPM, ControlMode.Velocity);

    /* LIMELIGHT AUTO AIM CODE, REINTRODUCE AFTER DRIVE PID IS GOOD
    if (DriveJoystick.getCameraOrient()) // direction switch
      intakeforward = !intakeforward;

    if (!intakeforward) {
      move = -1 * move;
      left = -left;
      right = -right;
    }
    accumError = Math.abs(x);
    oldTime = time;
    time = timer.get();
    oldX = x;
    //x = Limelight.getX() - 3;
    double deltaVelocity = (x - oldX) / (time - oldTime);
    power_LL = cP_LL * x + (cD_LL * deltaVelocity) + cI_LL * accumError; // The PID-based power calculation for LL
                                                                         // auto-aim
    isAligned = x < 6 && x > -6;
    SmartDashboard.putBoolean("Aim Aligned?", isAligned);
    SmartDashboard.putNumber("deltaVelocity", deltaVelocity);
    SmartDashboard.putNumber("PIDpower", power_LL);
    SmartDashboard.putNumber("cP_LL", cP_LL);
    SmartDashboard.putNumber("x", x);
    
    

    if (DriveJoystick.aim())
     // drive.arcadeDrive(0, power_LL);
     {}
    else
      move();
    if (DriveJoystick.aim())
      accumError = 0;
    */
  }
  
  
  public static void move() {
    //bootleg differential drive. needs testing.
    targetVLeft = (move * maxFtPerSec * fts_to_RPM); //+ turn * maxFtPerSec * fts_to_RPM;
    targetVRight = (-move * maxFtPerSec * fts_to_RPM); //- turn * maxFtPerSec * fts_to_RPM;
    //leftBack.set(TalonSRXControlMode.PercentOutput, targetVLeft / 6100);
    //rightBack.set(TalonSRXControlMode.PercentOutput, targetVRight / 6100);
    SmartDashboard.putNumber("Target V left", targetVLeft);
    SmartDashboard.putNumber("Target V right", targetVRight);

    leftBack.set(ControlMode.Velocity, targetVLeft);
    rightBack.set(ControlMode.Velocity, targetVRight);


    SmartDashboard.putNumber("left target", leftBack.getClosedLoopTarget());
    SmartDashboard.putNumber("right target", rightBack.getClosedLoopTarget());

    //leftBack.set(TalonSRXControlMode.Velocity, (move * maxFtPerSec * fts_to_RPM), DemandType.AuxPID, turn);
    //rightBack.set(TalonSRXControlMode.Velocity, (-move * maxFtPerSec * fts_to_RPM), DemandType.ArbitraryFeedForward, turn);

    /*
    if (driveSelection) {

     // drive.arcadeDrive(move, turn);
      driveMode = "Arcade Drive";
    } else {
     // drive.tankDrive(left, right);
      driveMode = "Tank Drive";
    }
    // drive.arcadeDrive(move, turn);
    */
  }
  
  public void run() {
    joystickDrive();
    
    //leftRPM = RobotMap.leftBack.getEncoder().getVelocity();
    //rightRPM = RobotMap.rightBack.getEncoder().getVelocity();
    //leftEncoderCounts = RobotMap.leftBack.getEncoder().getPosition();
    //rightEncoderCounts = RobotMap.rightBack.getEncoder().getPosition();
    //joystickDrive();
    SmartDashboard.putNumber("joy pos", DriveJoystick.getMove());
    //adjustPIDS();
    SmartDashboard.putBoolean("Intake Front?", intakeforward);
    SmartDashboard.putNumber("RPM Difference", (Math.abs(leftRPM) - Math.abs(rightRPM)));
    SmartDashboard.putBoolean("Drive Selection", driveSelection);
    SmartDashboard.putNumber("Target V", move * 12.21 * 409.3);

    SmartDashboard.putNumber("Left Velocity", leftBack.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right Velocity", rightBack.getSelectedSensorVelocity());

    SmartDashboard.putNumber("Left Percent", leftBack.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Percemt", rightBack.getMotorOutputPercent());

    //sends a 2d model of the field to shuffleboard
    //soon we use odometry to put a simulation of our robot on the field!
    SmartDashboard.putData(getField());
    
  }
  /*
  public static void autoRun(double startTime, double endTime, double moveSpeed, double turnSpeed) {
    //double time = Robot.timer.get();
    //if (time > startTime && time < endTime) {
      move = moveSpeed;
      turn = turnSpeed;
      move();
    //}
  }
  public static void testDrive(){
    //controller max ft/sec = 12.21
    //1 RPM on motor is 0.002443 ft/sec
    //leftPIDController.setReference((move * 12.21 * fts_to_RPM) + turn * 12.21 * fts_to_RPM, ControlType.kVelocity);
    //rightPIDController.setReference((-move * 12.21 * fts_to_RPM) - turn * 12.21 * fts_to_RPM, ControlType.kVelocity);
  }
  */
  
  /*
  public void adjustPIDS() { 
        if (calibrateJoy.getRawAxis(5) < -0.5) {
            cP_LL = cP_LL + 0.0001;
        } else if (calibrateJoy.getRawAxis(5) > 0.5) {
            cP_LL = cP_LL - 0.0001;
        }
        SmartDashboard.putNumber("cD_LL", cD_LL);
        if (calibrateJoy.getRawAxis(1) < -0.5) {
            cD_LL = cD_LL + 0.0001;
        } else if (calibrateJoy.getRawAxis(1) > 0.5) {
            cD_LL = cD_LL - 0.0001;
        }
        SmartDashboard.putNumber("cP_LL", cP_LL);

        if (calibrateJoy.getRawAxis(3) > 0.5) {
            cI_LL = cI_LL + 0.0001;
        } 
        if (calibrateJoy.getRawAxis(2) > 0.5) {
            cI_LL = cI_LL - 0.0001;
        }
        SmartDashboard.putNumber("cI_LL", cI_LL);
    }*/
    
  }
