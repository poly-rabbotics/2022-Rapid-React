
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Controls.DriveJoystick;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Drive {
  Timer timer;
  double time, oldTime, cP_LL, cD_LL, cI_LL; // Limelight PID control vars
  static double power_LL;
  double accumError;
  static double x;
  double oldX;
  static double fts_to_RPM;
  public static double leftRPM, rightRPM, leftEncoderCounts, rightEncoderCounts, initialLeftCounts, initialRightCounts;
  static double cP, cD, cI, leftPower, rightPower; // drive speed PID control vars
  static double leftSpeedError, rightSpeedError, leftSpeedSetpoint, rightSpeedSetpoint;
  static double move, turn, left, right; // move and turn for arcade drive, left and right for tank
  static double maxFtPerSec;
  Joystick calibrateJoy;
  public static TalonSRX leftBack, leftFront, rightBack, rightFront;
  public static String driveMode;
  boolean isAligned;
  static double targetVLeft;
  static double targetVRight;
  private Field2d field = new Field2d();
  public static boolean PIDDriveActive;
  public boolean highTorqueModeActive;
  boolean rotateInitialized, movementInitialized;
  public boolean movementCompleted, turnCompleted;
  Rotation2d gyroToRadians;
  DifferentialDriveOdometry odometry;
  static AHRSGyro gyro;
  double positionSetpoint;
  double turnError;
  double targetAngle;
  double gyroAngle;
  public static double ENCODER_COUNTS_PER_360, ENCODER_COUNTS_PER_INCH;
  double moveSetpoint;
  double currPositionL,currPositionR;
  static Pigeon2 pigeon;

  private static final double PID_DEADZONE = 10;

  public Drive() {
    ENCODER_COUNTS_PER_360 = 108200;  //300 per degree
    ENCODER_COUNTS_PER_INCH = 1450;
    calibrateJoy = new Joystick(2);
    currPositionL=0;
    currPositionR=0;
    timer = new Timer();
    timer.start();
    cP_LL = 0.015; // old LL PIDs
    cD_LL = 0.0025;
    cI_LL = 0.0012;
    // leftSpeedSetpoint = 0;
    // rightSpeedSetpoint = 0;
    move = 0;
    turn = 0;
    oldTime = 0;
    accumError = 0;
    // time = timer.get();
    x = 0;
    driveMode = "";

    gyro = new AHRSGyro();

    leftEncoderCounts = 0;
    rightEncoderCounts = 0;

    targetVLeft = 0;
    targetVRight = 0;

    PIDDriveActive = false;
    highTorqueModeActive = false;

    leftBack = RobotMap.leftBack;
    leftFront = RobotMap.leftFront;
    rightBack = RobotMap.rightBack;
    rightFront = RobotMap.rightFront;

    leftBack.configFactoryDefault();
    rightBack.configFactoryDefault();

    leftFront.follow(leftBack);
    rightFront.follow(rightBack);
    leftFront.setInverted(InvertType.FollowMaster);
    rightFront.setInverted(InvertType.FollowMaster);
    leftBack.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
    rightBack.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);

    fts_to_RPM = 409.3;
    maxFtPerSec = 60; // find this out through testing NOT ACTUALLY MAX FEET PER SECOND

    leftBack.getSensorCollection().setAnalogPosition(0, 30);
    rightBack.getSensorCollection().setAnalogPosition(0, 30);
    leftBack.setSelectedSensorPosition(0);
    rightBack.setSelectedSensorPosition(0);
    // ENCODER COUNTS PER FOOT: 13445
    // Counts per inch: 1120

    pigeon = new Pigeon2(9);

    rotateInitialized = false;
    /*
     * DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
     * AHRSGyro.getDegrees(), new Pose2d(5.0, 13.5, new Rotation2d()));
     */
    gyroToRadians = Rotation2d.fromDegrees(gyro.getDegrees());
    odometry = new DifferentialDriveOdometry(gyroToRadians, new Pose2d(0, 0, new Rotation2d()));
    RobotMap.drivePancake.set(Value.kForward); //speed mode
    initAutoDrive();
  }

  public static boolean isAutoDrive = false;
  public boolean intakeforward = true;
  // static SpeedControllerGroup leftDrive = new
  // SpeedControllerGroup(RobotMap.leftFront, RobotMap.leftBack);
  // static SpeedControllerGroup rightDrive = new
  // SpeedControllerGroup(RobotMap.rightFront, RobotMap.rightBack);
  static boolean driveSelection = DriveJoystick.driveMode();
  // public static DifferentialDrive drive = new DifferentialDrive(leftDrive,
  // rightDrive);

  public Field2d getField() {
    return field;
  }

  public DifferentialDriveOdometry updateOdometry() {
    gyroToRadians = Rotation2d.fromDegrees(gyro.getDegrees());
    odometry.update(gyroToRadians, leftEncoderCounts * 2597, rightEncoderCounts * 2597);
    return odometry;
  }

  void joystickDrive() {
    driveModeSet();

    // DRIVE GEAR SHIFTING PANCAKES //

    if (highTorqueModeActive) {
      RobotMap.drivePancake.set(Value.kReverse); //REVERSE IS TORQUE MODE
    } else
      RobotMap.drivePancake.set(Value.kForward); //FORWARD IS SPEED MODE

    SmartDashboard.putBoolean("High Torque Mode?", highTorqueModeActive);

    move = DriveJoystick.getMove();
    turn = DriveJoystick.getTurn();
    left = -DriveJoystick.getMove();
    right = -DriveJoystick.axisFive();

    if (DriveJoystick.driveMode()) //keep for potential future use
      driveSelection = !driveSelection; //unused, previously switched between arcade drive and tank drive styles

    move = Math.signum(move) * Math.pow(move, 2); // DRIVE CURVES
    turn = Math.pow(turn, 1) * 0.4;

    if ((move < 0.1) && (move > -0.1)) { // JOYSTICK DEADZONE
      move = 0;
    }
    if ((turn < 0.15) && (turn > -0.15)) { // JOYSTICK DEADZONE
      turn = 0;
    }

    if (intakeforward) {
      move = -1 * move;
      left = -left;
      right = -right;
    } else if (!intakeforward) {
      move = 1 * move;
    }

    if (DriveJoystick.getPreciseFront()) { //SLOW MODE
      move = 0.25;
    } else if (DriveJoystick.getPreciseBack()) {
      move = -0.25;
    }
    if (DriveJoystick.getPreciseRight()) {
      turn = 0.25;
    } else if (DriveJoystick.getPreciseLeft()) {
      turn = -0.25;
    }
    move();

    if (DriveJoystick.getCameraOrient()) // direction switch
      intakeforward = !intakeforward;

    // LIMELIGHT AUTO AIM CODE, WORKS WELL!
    accumError = Math.abs(x);
    oldTime = time;
    time = timer.get();
    oldX = x;
    x = RobotMap.limelight.getX();
    double deltaVelocity = (x - oldX) / (time - oldTime);
    power_LL = cP_LL * x + (cD_LL * deltaVelocity) + cI_LL * accumError; // The PID-based power calculation for LL

    /*
    if (Limelight.limelightProfile == 2) { //inverts LL values when it is upside down 
      power_LL = power_LL * -1;
    } */
    isAligned = x < 6 && x > -6;
    SmartDashboard.putBoolean("Aim Aligned?", isAligned);
    SmartDashboard.putNumber("deltaVelocity", deltaVelocity);
    SmartDashboard.putNumber("PIDpower", power_LL);
    SmartDashboard.putNumber("cP_LL", cP_LL);
    SmartDashboard.putNumber("x", x);

    if(PIDDriveActive) {
      autoBalance();
    }
  }

  public static void move() {
    // bootleg differential drive. needs testing.
    // - turn * maxFtPerSec * fts_to_RPM;
    // leftBack.set(TalonSRXControlMode.PercentOutput, targetVLeft / 6100);
    // rightBack.set(TalonSRXControlMode.PercentOutput, targetVRight / 6100);

    //SETS TARGET VELOCITIES FOR PID MODE
    if (DriveJoystick.aim()) { //IF LIMELIGHT AUTO AIM BUTTON PRESSED, TURN COMMANDED BY LL DATA
      targetVLeft = (move * maxFtPerSec * fts_to_RPM - (power_LL * maxFtPerSec * fts_to_RPM)); 
      targetVRight = (-move * maxFtPerSec * fts_to_RPM - (power_LL * maxFtPerSec * fts_to_RPM));
    } else { //IF LLAA NOT PRESSED, JOYSTICK TURN CONTROL
      targetVLeft = (move * maxFtPerSec * fts_to_RPM - (turn * maxFtPerSec * fts_to_RPM));
      targetVRight = (-move * maxFtPerSec * fts_to_RPM - (turn * maxFtPerSec * fts_to_RPM));
    }

    if (PIDDriveActive) {
      /* COMMENTED OUT TO REPLACE WITH AUTO BALANCING CODE
      leftBack.set(ControlMode.Velocity, targetVLeft);
      rightBack.set(ControlMode.Velocity, targetVRight);*/
    } else {
      if (DriveJoystick.aim()) {
        
        leftBack.set(ControlMode.PercentOutput, move - x * 0.01667);
        rightBack.set(ControlMode.PercentOutput, -move - x * 0.01667);
        

        leftBack.set(ControlMode.PercentOutput, move - power_LL);
        rightBack.set(ControlMode.PercentOutput, -move - power_LL);
      } else {
        leftBack.set(ControlMode.PercentOutput, move - turn);
        rightBack.set(ControlMode.PercentOutput, -move - turn);
      }
    }

    

    /*
     * leftBack.set(ControlMode.PercentOutput, move + turn);
     * rightBack.set(ControlMode.PercentOutput, -move - turn);
     * 
     * leftBack.set(ControlMode.Velocity, targetVLeft);
     * rightBack.set(ControlMode.Velocity, targetVRight);
     */

    SmartDashboard.putNumber("Target V left", targetVLeft);
    SmartDashboard.putNumber("Target V right", targetVRight);

    SmartDashboard.putNumber("Move Joystick", move);
    SmartDashboard.putNumber("Turn Joystick", turn);

  }

  public void driveModeSet() { //SETS DRIVE MODE: PID OR PERCENTOUTPUT, TORQUE OR SPEED
    PIDDriveActive = DriveJoystick.enablePIDMode();
    if (PIDDriveActive) {
      initPIDDrive();
    } else if (!PIDDriveActive) {
      initPercentOutputDrive();
    }
    if (DriveJoystick.getToggleGears()) {
      highTorqueModeActive = !highTorqueModeActive;
    }

  }
  public void initAutoDrive() {
    // ALL THE INITIALIZATION FOR SETTING UP PID CONTROL MODE
    leftBack.configFactoryDefault();
    leftBack.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
    leftBack.configNominalOutputForward(0);
    leftBack.configNominalOutputReverse(0);
    leftBack.configPeakOutputForward(0.4);
    leftBack.configPeakOutputReverse(-0.4);
    leftBack.setSensorPhase(false);
    leftBack.config_kP(0, .15);
    leftBack.config_kI(0, 0.0);
    leftBack.config_kD(0, .000);
    leftBack.config_kF(0, 0);
    leftBack.selectProfileSlot(0, 0);

    rightBack.configFactoryDefault();
    rightBack.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
    rightBack.configNominalOutputForward(0);
    rightBack.configNominalOutputReverse(0);
    rightBack.configPeakOutputForward(0.4);
    rightBack.configPeakOutputReverse(-0.4); //for some reason this was 0.6, -0.6 previously, meaning that the limit for left was different than right???
    rightBack.setSensorPhase(false);
    // THESE PIDS WORK WELL
    rightBack.config_kP(0, .15);
    rightBack.config_kI(0, 0.0);
    rightBack.config_kD(0, .000);
    rightBack.config_kF(0, 0);
    rightBack.selectProfileSlot(0, 0);

  }
  public void initPIDDrive() {
    // ALL THE INITIALIZATION FOR SETTING UP PID CONTROL MODE
    leftBack.configFactoryDefault();
    leftBack.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
    leftBack.configNominalOutputForward(0);
    leftBack.configNominalOutputReverse(0);
    leftBack.configPeakOutputForward(1);
    leftBack.configPeakOutputReverse(-1);
    leftBack.setSensorPhase(false);
    leftBack.config_kP(0, .1);
    leftBack.config_kI(0, 0.0005);
    leftBack.config_kD(0, .0001);
    leftBack.config_kF(0, 0);
    leftBack.selectProfileSlot(0, 0);

    rightBack.configFactoryDefault();
    rightBack.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
    rightBack.configNominalOutputForward(0);
    rightBack.configNominalOutputReverse(0);
    rightBack.configPeakOutputForward(1);
    rightBack.configPeakOutputReverse(-1);
    rightBack.setSensorPhase(false);
    // THESE PIDS WORK WELL
    rightBack.config_kP(0, .1);
    rightBack.config_kI(0, .0005);
    rightBack.config_kD(0, .0001);
    rightBack.config_kF(0, 0);
    rightBack.selectProfileSlot(0, 0);

  }

  public void initPercentOutputDrive() { //Sets motors to be ready for percent output drive
    leftBack.configFactoryDefault();
    rightBack.configFactoryDefault();
  }

  public void run() {
    //adjustPIDS();
    joystickDrive();
    driveModeSet();
    leftEncoderCounts = leftBack.getSelectedSensorPosition();
    rightEncoderCounts = -rightBack.getSelectedSensorPosition();
    SmartDashboard.putNumber("left Encoder Feet", leftEncoderCounts / 13445);
    SmartDashboard.putNumber("right Encoder Feet", rightEncoderCounts / 13445);
    SmartDashboard.putNumber("left Encoder Counts", leftEncoderCounts);
    SmartDashboard.putNumber("right Encoder Counts", rightEncoderCounts);
    SmartDashboard.putNumber("left Encoder Degrees", leftEncoderCounts / 108200 / 360);
    SmartDashboard.putNumber("right Encoder Degrees", rightEncoderCounts / 108200 / 360);
    SmartDashboard.putNumber("joy pos", DriveJoystick.getMove());
    SmartDashboard.putBoolean("Intake Front?", intakeforward);
    SmartDashboard.putNumber("RPM Difference", (Math.abs(leftRPM) - Math.abs(rightRPM)));
    SmartDashboard.putBoolean("Drive Selection", driveSelection);
    SmartDashboard.putNumber("Target V", move * 12.21 * 409.3);

    SmartDashboard.putNumber("Left Velocity", leftBack.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right Velocity", rightBack.getSelectedSensorVelocity());

    SmartDashboard.putNumber("Left Percent", leftBack.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Percent", rightBack.getMotorOutputPercent());

    SmartDashboard.putBoolean("High Torque Mode Active?", highTorqueModeActive);
    SmartDashboard.putBoolean("PID Drive Mode Active", PIDDriveActive);

    SmartDashboard.putData(getField());
  }

  public void resetEncoders() { //SETS THE ENCODER COUNTS TO ZERO BOTH SIDES
    //leftBack.getSensorCollection().setAnalogPosition(0, 30);
    //rightBack.getSensorCollection().setAnalogPosition(0, 30);
    leftBack.setSelectedSensorPosition(0);
    rightBack.setSelectedSensorPosition(0);
    
  }
  public boolean moveByInches(double startTime, double endTime, double inches) { //AUTONOMOUS DRIVE METHOD
    double time = Robot.autoTimer.get();
    if (!movementInitialized) {
      movementInitialized = true;
      movementCompleted = false;
      moveSetpoint = inches * ENCODER_COUNTS_PER_INCH;
      initPIDDrive();
      resetEncoders();
    }
    if (time < endTime && time > startTime && !movementCompleted) {
      leftBack.set(ControlMode.Position, moveSetpoint);
      rightBack.set(ControlMode.Position, -moveSetpoint);
      if ((leftBack.getSelectedSensorPosition() < moveSetpoint + 1000) && (leftBack.getSelectedSensorPosition() > moveSetpoint - 1000) && !movementCompleted) {
        movementCompleted = true;
        resetEncoders();
      } else movementCompleted = false;
    }
    
    return movementCompleted;
  }

  public boolean moveByInchesBasic(double startTime, double endTime, double inches) { //AUTONOMOUS DRIVE METHOD
    double time = Robot.autoTimer.get();
    double moveSetpointL = inches*ENCODER_COUNTS_PER_INCH+currPositionL;
    double moveSetpointR = inches*ENCODER_COUNTS_PER_INCH-currPositionR;
    
    if (time < endTime && time > startTime && !movementCompleted) {
      //initPIDDrive();
      leftBack.set(ControlMode.Position, moveSetpointL);
      rightBack.set(ControlMode.Position, -moveSetpointR);

      if ((leftBack.getSelectedSensorPosition() < moveSetpointL + 1000) && (leftBack.getSelectedSensorPosition() > moveSetpointL - 1000))
      {
        movementCompleted=true;
        currPositionL=leftBack.getSelectedSensorPosition();
        currPositionR=rightBack.getSelectedSensorPosition();
      }
    }
    return movementCompleted;
  }

    public boolean turnByDegreesBasic2(double startTime, double endTime, double inches) { //AUTONOMOUS DRIVE METHOD
      double time = Robot.autoTimer.get();
      double moveSetpointL = inches*ENCODER_COUNTS_PER_360/360.0+currPositionL;
      double moveSetpointR = inches*ENCODER_COUNTS_PER_360/360.0-currPositionR;
      
      if (time < endTime && time > startTime && !movementCompleted) {
        //initPIDDrive();
        leftBack.set(ControlMode.Position, moveSetpointL);
        rightBack.set(ControlMode.Position, moveSetpointR);
  
        if ((leftBack.getSelectedSensorPosition() < moveSetpointL + 1000) && (leftBack.getSelectedSensorPosition() > moveSetpointL - 1000))
        {
          movementCompleted=true;
          currPositionL=leftBack.getSelectedSensorPosition();
          currPositionR=rightBack.getSelectedSensorPosition();
        }
      }


    
    return movementCompleted;
  }

  public void setMoveInit(double startTime, double endTime) {
    double time = Robot.autoTimer.get();
    if (time < endTime && time > startTime) {
      movementInitialized = false;
      movementCompleted= false;
      initAutoDrive();
    }

  }

  public void resetEncodersCall(double startTime, double endTime) {
    double time = Robot.autoTimer.get();
    if (time < endTime && time > startTime) {
      resetEncoders();
    }

  }

  public boolean turnByDegreesBasic(double startTime, double endTime, double finalAngle) {
    // NO GYRO TURN CORRECTION
    double time = Robot.autoTimer.get();
    double initialPosition = leftBack.getSelectedSensorPosition();
    SmartDashboard.putNumber("position setpoint turn", positionSetpoint);
    SmartDashboard.putBoolean("rotate initialized?", rotateInitialized);

    if (time < endTime && time > startTime) {
      if (!rotateInitialized) {
        rotateInitialized = true;
        positionSetpoint =  ENCODER_COUNTS_PER_360/360 * finalAngle;
        leftBack.set(ControlMode.Position, positionSetpoint * ENCODER_COUNTS_PER_360/360);
        rightBack.set(ControlMode.Position, positionSetpoint * ENCODER_COUNTS_PER_360/360);
      }

      if ((leftBack.getSelectedSensorPosition() > positionSetpoint - 100 
      && leftBack.getSelectedSensorPosition() < positionSetpoint + 100) && !turnCompleted) {
        resetEncoders();
        turnCompleted = true;
      } else if (!turnCompleted) {
        
        turnCompleted = false;
      }
    } else {
    }
    return turnCompleted;
  }
  public boolean turnByDegrees(double startTime, double endTime, double finalAngle) { //AUTONOMOUS TURN METHOD
    double time = Robot.autoTimer.get();
    double initialPosition = leftBack.getSelectedSensorPosition();

    
    if (time < endTime && time > startTime) {

    if (!rotateInitialized) {
      rotateInitialized = true;
      targetAngle = finalAngle;
      positionSetpoint = initialPosition + ENCODER_COUNTS_PER_360/360 * finalAngle;
      //resetEncoders();
      gyro.reset();
    }

    if (leftBack.getSelectedSensorPosition() > positionSetpoint - 100
        && leftBack.getSelectedSensorPosition() < positionSetpoint + 100) {
      gyroAngle = gyro.getDegrees();
      if ((gyroAngle < (finalAngle - 1)) && (gyroAngle > (finalAngle + 1))) {
        rotateInitialized = false;
        resetEncoders();
        return true;
      } else {
        targetAngle = finalAngle - gyroAngle;
        resetEncoders();
        gyro.reset();
        return false;
      }
    } else { // Encoder target not yet reached
      leftBack.set(ControlMode.Position, targetAngle * ENCODER_COUNTS_PER_360/360);
      rightBack.set(ControlMode.Position, targetAngle * ENCODER_COUNTS_PER_360/360);
      return false;
    }

    }
    else return true; //this is the timeout return for when allotted time runs out and action has not completed

  }

  public boolean goToHeading(double startTime, double endTime, double finalAngle) {
    double difference = finalAngle - gyro.getDegrees();
    boolean targetReached = Math.abs(difference) < 2;
    double kP = 1.0/60.0;
    SmartDashboard.putBoolean("target reached?", targetReached);
    SmartDashboard.putNumber("difference", difference);
    double time = Robot.autoTimer.get();

    if (time < endTime && time > startTime) {
      initPercentOutputDrive();
    if (!targetReached) {
      leftBack.set(ControlMode.PercentOutput, -difference * kP);
      rightBack.set(ControlMode.PercentOutput, -difference * kP);
      currPositionL=leftBack.getSelectedSensorPosition();
      currPositionR=rightBack.getSelectedSensorPosition();

    } else {
     // movementCompleted=true;
     // currPositionL=leftBack.getSelectedSensorPosition();
     // currPositionR=rightBack.getSelectedSensorPosition();
    }

    }
    return false;
  }
  

  public boolean goToEncCounts(double startTime, double endTime, double count) {
    double difference = count - leftBack.getSelectedSensorPosition();
    boolean targetReached = Math.abs(difference) < 1000;
    double kP = 1.0/100000.0;
    double power = difference*kP;
    if(power>0.5)
      power=0.5;
    if(power<-.5)
      power=-0.5;
    SmartDashboard.putBoolean("target reached?", targetReached);
    SmartDashboard.putNumber("difference", difference);
    double time = Robot.autoTimer.get();

    if (time < endTime && time > startTime) {
      initPercentOutputDrive();
    if (!targetReached) {
      leftBack.set(ControlMode.PercentOutput, power);
      rightBack.set(ControlMode.PercentOutput, -power);

    } else {
      resetEncoders();
      leftBack.set(ControlMode.PercentOutput, 0);
      rightBack.set(ControlMode.PercentOutput, 0);
     // movementCompleted=true;
     // currPositionL=leftBack.getSelectedSensorPosition();
     // currPositionR=rightBack.getSelectedSensorPosition();
    }

    }
    return false;
  }

  public boolean goToEncCountsTurn(double startTime, double endTime, double count) {
    double difference = count - leftBack.getSelectedSensorPosition();
    boolean targetReached = Math.abs(difference) < 1000;
    double kP = 1.0/20000.0;
    double power = difference*kP;
    if(power>.5)
      power=0.5;
    if(power<-.5)
      power=-0.5;
    SmartDashboard.putBoolean("target reached?", targetReached);
    SmartDashboard.putNumber("difference", difference);
    double time = Robot.autoTimer.get();

    if (time < endTime && time > startTime) {
      initPercentOutputDrive();
    if (!targetReached) {
      leftBack.set(ControlMode.PercentOutput, power);
      rightBack.set(ControlMode.PercentOutput, power);

    } else {
     // resetEncoders();
     // leftBack.set(ControlMode.PercentOutput, 0);
      //rightBack.set(ControlMode.PercentOutput, 0);
     // movementCompleted=true;
     // currPositionL=leftBack.getSelectedSensorPosition();
     // currPositionR=rightBack.getSelectedSensorPosition();
    }

    }
    return false;
  }


  public void stopMotors(double startTime, double endTime) {
    double time = Robot.autoTimer.get();

    if (time < endTime && time > startTime) {
      leftBack.set(ControlMode.PercentOutput, 0);
      rightBack.set(ControlMode.PercentOutput, 0);

    }
  }


/*
 * public static void autoRun(double startTime, double endTime, double
 * moveSpeed, double turnSpeed) {
 * //double time = Robot.timer.get();
 * //if (time > startTime && time < endTime) {
 * move = moveSpeed;
 * turn = turnSpeed;
 * move();
 * //}
 * }
 * public static void testDrive(){
 * //controller max ft/sec = 12.21
 * //1 RPM on motor is 0.002443 ft/sec
 * //leftPIDController.setReference((move * 12.21 * fts_to_RPM) + turn * 12.21 *
 * fts_to_RPM, ControlType.kVelocity);
 * //rightPIDController.setReference((-move * 12.21 * fts_to_RPM) - turn * 12.21
 * * fts_to_RPM, ControlType.kVelocity);
 * }
 */

  private void autoBalance() {
    
    if (getGlobalRotation() <= -PID_DEADZONE) { //if robot is tilted forwards
      targetVLeft = 30;
      targetVRight = 30;
    } else if (getGlobalRotation() >= PID_DEADZONE) { //if robot is tilted backwards
      targetVLeft = -30;
      targetVRight = -30;
    } else { //if robot is not tilted
      targetVLeft = 0;
      targetVRight = 0;
    }
    

    leftBack.set(ControlMode.Velocity, targetVLeft);
    rightBack.set(ControlMode.Velocity, targetVRight);
  }

  private double getGlobalRotation() {
    return pigeon.getPitch() % 360.0;
  }


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
}

}


