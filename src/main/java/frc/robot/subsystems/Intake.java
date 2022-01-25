// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.revrobotics.CANSparkMax;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Controls.MechanismsJoystick;

/** Add your docs here. */
//public class intake extends Subsystem {
public class Intake {
  
    static PWMVictorSPX intakeWinch;
    static CANSparkMax intakeMotor;
    static double intakeSpeed;
    
  public Intake() {
    intakeMotor = RobotMap.intakeMotor;
    intakeSpeed = 0.0;
  }
  public void run() {
    if (MechanismsJoystick.intakeSpeedIncrease()) {
      SmartDashboard.putBoolean("speed increase", true);
      intakeSpeed += 0.05;
  } else SmartDashboard.putBoolean("speed increase", false);


  if (MechanismsJoystick.intakeSpeedDecrease()) {
      SmartDashboard.putBoolean("speed decrease", true);
      intakeSpeed -= 0.05;
  } else SmartDashboard.putBoolean("speed decrease", false);


 

  if (MechanismsJoystick.intakeButton()) {
      intakeMotor.set(intakeSpeed);
  } else intakeMotor.set(0);

  SmartDashboard.putNumber("intake speed setpoint", intakeSpeed);
  SmartDashboard.putNumber("intake RPM", intakeMotor.getEncoder().getVelocity());

    //intakePneumatics();

    SmartDashboard.putNumber("intake speed", intakeSpeed);
  }
}
/*
public static void intakePneumatics() {
    if(MechanismsJoystick.arm()) {
        RobotMap.intakePiston.set(Value.kForward);
    }
    
    else if(!MechanismsJoystick.arm()) {
        RobotMap.intakePiston.set(Value.kReverse);
    }
}
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
}
*/

