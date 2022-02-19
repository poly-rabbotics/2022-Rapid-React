// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.revrobotics.CANSparkMax;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import frc.robot.Robot;

import com.revrobotics.CANSparkMax;

import frc.robot.Controls.MechanismsJoystick;

/** Add your docs here. */
//public class intake extends Subsystem {
public class Intake {
    static PWMVictorSPX intakeWinch;
    static CANSparkMax intake;
    static double intakeSpeed;
    static boolean intakeDown;
    static double intakeWinchPower = 0;

  public Intake() {
    
    intake = RobotMap.intakeMotor;
    intakeSpeed = 0.8;
  }
  public void run() {
    intakeSpeed = 0.8;
    if (MechanismsJoystick.reverse()) {
      intakeSpeed = -intakeSpeed;
    }

    if (MechanismsJoystick.intake()) {
      intake.set(intakeSpeed);
    } else intake.set(0);

    //intakePneumatics();
  }

  public static void autoRun(double startTime, double endTime, double intakeSpeed) {
    double time = Robot.timer.get();
    if (time > startTime && time < endTime) {
      intake.set(intakeSpeed);
    }
  }

  public static void deployIntake(double startTime, double endTime, boolean isDeployed) {
    double time = Robot.timer.get();
    if (time > startTime && time < endTime) {
      if (isDeployed) {
        RobotMap.intakeSolenoid.set(Value.kForward);
      } else {
        RobotMap.intakeSolenoid.set(Value.kReverse);
      }
      
    }
  }

//public static void intakePneumatics() {
    //if(MechanismsJoystick.toggleIntakePiston()) {
        //RobotMap.intakeSolenoid.set(Value.kForward);
    //}
    
    //else if(!MechanismsJoystick.toggleIntakePiston()) {
       // RobotMap.intakeSolenoid.set(Value.kReverse);
    //}
//}

}

