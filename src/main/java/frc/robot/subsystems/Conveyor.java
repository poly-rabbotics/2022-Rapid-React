// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.revrobotics.CANSparkMax;

import frc.robot.RobotMap;
import com.revrobotics.CANSparkMax;
import frc.robot.Controls.MechanismsJoystick;
import frc.robot.Robot;

public class Conveyor {
    
    static boolean reversed;
    static CANSparkMax conveyorMotor;
    
  public Conveyor() {
    conveyorMotor = RobotMap.conveyorMotor;
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
  } else conveyorMotor.set(0);
  }

  public static void autoRun(double startTime, double endTime, double conveyorSpeed) {
    double time = Robot.timer.get();
    if (time > startTime && time < endTime) {
      conveyorMotor.set(conveyorSpeed);
    }
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

