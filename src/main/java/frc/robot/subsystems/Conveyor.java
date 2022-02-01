// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.revrobotics.CANSparkMax;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Controls.MechanismsJoystick;

public class Conveyor {
    
    public static CANSparkMax conveyorMotor;

    static boolean reversed;
    static CANSparkMax ConveyorMotor;
    
  public Conveyor() {
    conveyorMotor = new CANSparkMax(7, MotorType.kBrushless);
    reversed = false;
  }
  public void run() {
    if (MechanismsJoystick.conveyorReverse()) {
      if(reversed) reversed = false;
      else if(!reversed) reversed = true;   
    }

    if(MechanismsJoystick.conveyor())


 

  if (MechanismsJoystick.runConveyor()) {
      if(!reversed) conveyorMotor.set(1);
      if(reversed) conveyorMotor.set(-1);
  } else ConveyorMotor.set(0);
  }
}
/*
public static void ConveyorPneumatics() {
    if(MechanismsJoystick.arm()) {
        RobotMap.ConveyorPiston.set(Value.kForward);
    }
    
    else if(!MechanismsJoystick.arm()) {
        RobotMap.ConveyorPiston.set(Value.kReverse);
    }
}
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
}
*/

