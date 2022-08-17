// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.revrobotics.CANSparkMax;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import frc.robot.Robot;

import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Controls.DriveJoystick;
import frc.robot.Controls.GuitarJoystick;
import frc.robot.Controls.MechanismsJoystick;

/** Add your docs here. */
//public class intake extends Subsystem {
public class Intake {
    static PWMVictorSPX intakeWinch;
    static CANSparkMax intake;
    static double intakeSpeed;
    static boolean intakeDown;
    static double intakeWinchPower = 0;
    static DoubleSolenoid intakeSolenoid;
    static boolean pneumaticsTransitioning;

  public Intake() {
    intakeSolenoid = RobotMap.intakeSolenoid;
    intake = RobotMap.intakeMotor;
    intakeSpeed = -1;
    intake.setIdleMode(IdleMode.kBrake);
    pneumaticsTransitioning = false;
  }
  public void run() {
    
    if ((DriveJoystick.runIntake() || GuitarJoystick.intake()) && intakeSolenoid.get() == Value.kForward) {
      intake.set(intakeSpeed);
    } else if (DriveJoystick.runIntakeReverse() && intakeSolenoid.get() == Value.kForward) {
      intake.set(-intakeSpeed);
    } else intake.set(0);

    intakePneumatics();
  }

  public void autoRun(double startTime, double endTime, double intakeSpeed) {
    double time = Robot.autoTimer.get();
    if (time > startTime && time < endTime) {
      intake.set(intakeSpeed);
    }
  }

  public void deployIntake(double startTime, double endTime, boolean isDeployed) {
    double time = Robot.autoTimer.get();
    if (time > startTime && time < endTime) {
      if (isDeployed) {
        RobotMap.intakeSolenoid.set(Value.kForward);
      } else {
        RobotMap.intakeSolenoid.set(Value.kReverse);
      }
      
    }
  }

public static void intakePneumatics() {
    if(DriveJoystick.toggleIntakePiston() && !pneumaticsTransitioning) {
      pneumaticsTransitioning = true;
        if (RobotMap.intakeSolenoid.get() == Value.kForward) {
          RobotMap.intakeSolenoid.set(Value.kReverse);
        }
        else RobotMap.intakeSolenoid.set(Value.kForward);

    } 
    if (!DriveJoystick.toggleIntakePiston()) pneumaticsTransitioning = false;
    
}

}

