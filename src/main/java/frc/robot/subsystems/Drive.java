/*
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Controls.DriveJoystick;
import frc.robot.RobotMap;

/** Add your docs here. */

//GEAR RATIO OF 10.71 TO 1 IN DRIVETRAIN GEARBOXES
/*
public class Drive { 
  Timer timer;
  double time, oldTime, cP_LL, cD_LL, cI_LL, power_LL, accumError; //Limelight PID control vars
  double x, oldX;
  static double fts_to_RPM;
  static double leftRPM, rightRPM, leftEncoderCounts, rightEncoderCounts;
  static double cP, cD, cI, leftPower, rightPower; // drive speed PID control vars
  static double leftSpeedError, rightSpeedError, leftSpeedSetpoint, rightSpeedSetpoint;
  static double move, turn, left, right; // move and turn for arcade drive, left and right for tank
  Joystick calibrateJoy;
  static TalonFX leftBack, rightBack, leftFront, rightFront;
  static PIDController leftPIDController, rightPIDController;
  public static String driveMode;
  boolean isAligned;

  public Drive() {
    calibrateJoy = new Joystick(2);
    timer = new Timer();
    timer.start();
    cP_LL = 0.0425; // Constants determined through testing, don't change these
    cD_LL = 0.0173;
    cI_LL = 0.0014;
    leftSpeedError = 0;
    rightSpeedError = 0;
    leftSpeedSetpoint = 0;
    rightSpeedSetpoint = 0;
    move = 0;
    turn = 0;
    oldTime = 0;
    accumError = 0;
    time = timer.get();
    x = 0;
    driveMode = "";
    leftBack = RobotMap.leftBack;
    leftFront = RobotMap.leftFront;
    rightBack = RobotMap.rightBack;
    rightFront = RobotMap.rightFront;
    //leftPIDController =                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           , value);;
    //rightPIDController = rightBack.set();
    leftFront.follow(leftBack);
    rightFront.follow(rightBack);
    fts_to_RPM = 409.3;
    leftPIDController.setP(0.0002); //THESE PID VALUES LOOK GOOD OVERALL BUT COULD USE SOME OPTIMIZATION
    rightPIDController.setP(0.0002);
    leftPIDController.setD(0.0);
    rightPIDController.setD(0.0000);
    leftPIDController.setI(0.000001);
    rightPIDController.setI(0.000001);
  }
  public static boolean isAutoDrive = false;
  public boolean intakeforward = true;
  //static SpeedControllerGroup leftDrive = new SpeedControllerGroup(RobotMap.leftFront, RobotMap.leftBack);
  //static SpeedControllerGroup rightDrive = new SpeedControllerGroup(RobotMap.rightFront, RobotMap.rightBack);
  static boolean driveSelection = DriveJoystick.driveMode();
  //public static DifferentialDrive drive = new DifferentialDrive(leftDrive, rightDrive);

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

  }

  public static void move() {
    if (driveSelection) {

     // drive.arcadeDrive(move, turn);
      driveMode = "Arcade Drive";
    } else {
     // drive.tankDrive(left, right);
      driveMode = "Tank Drive";
    }
    // drive.arcadeDrive(move, turn);
  }

  public void run() {
    move = DriveJoystick.getMove();
    if ((move<0.05) || (move>-0.05)) { //deadzone between -5% and 5%
      move = 0;
    }
    //leftRPM = RobotMap.leftBack.getEncoder().getVelocity();
    //rightRPM = RobotMap.rightBack.getEncoder().getVelocity();
    //leftEncoderCounts = RobotMap.leftBack.getEncoder().getPosition();
    //rightEncoderCounts = RobotMap.rightBack.getEncoder().getPosition();
    //joystickDrive();
    testDrive();
    SmartDashboard.putNumber("joy pos", DriveJoystick.getMove());
    adjustPIDS();
    SmartDashboard.putBoolean("Intake Front?", intakeforward);
    SmartDashboard.putNumber("RPM Difference", (Math.abs(leftRPM) - Math.abs(rightRPM)));
    SmartDashboard.putBoolean("Drive Selection", driveSelection);
    SmartDashboard.putNumber("Target V", move * 12.21 * 409.3);
  }

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
    //THIS WORKS!
    //leftPIDController.setReference((move * 12.21 * fts_to_RPM) + turn * 12.21 * fts_to_RPM, ControlType.kVelocity);
    //rightPIDController.setReference((-move * 12.21 * fts_to_RPM) - turn * 12.21 * fts_to_RPM, ControlType.kVelocity);
  }

  
  
  public void adjustPIDS() { //use for adjusting PID values LIMELIGHT ONLY
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
*/