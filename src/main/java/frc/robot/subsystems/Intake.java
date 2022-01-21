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
    static CANSparkMax intake;
    static double intakeSpeed;
    static boolean intakeDown;
    static double intakeWinchPower = 0;

  public Intake() {
    intake = RobotMap.intake;
    intakeSpeed = 0.0;
  }
  public void run() {
    /*intakeSpeed = -0.8;
    if (MechanismsJoystick.reverse()) {
      intakeSpeed = -intakeSpeed;
    }

    if (MechanismsJoystick.intake()) {
      intake.set(intakeSpeed);
    } else intake.set(0);
    */

    if (MechanismsJoystick.intakeUp()) {
      SmartDashboard.putBoolean("intake speed increase", true);
      intakeSpeed += 0.05;
  } else SmartDashboard.putBoolean("intake speed increase", false);


  if (MechanismsJoystick.intakeDown()) {
      SmartDashboard.putBoolean("intake speed decrease", true);
      intakeSpeed -= 0.05;
  } else SmartDashboard.putBoolean("intake speed decrease", false);

    if (MechanismsJoystick.intakeRun()) {
      intake.set(intakeSpeed);
    } else intake.set(0);

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
