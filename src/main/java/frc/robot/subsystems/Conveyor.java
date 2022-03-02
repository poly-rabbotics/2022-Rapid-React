// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.revrobotics.CANSparkMax;

import frc.robot.RobotMap;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Controls.MechanismsJoystick;
import frc.robot.Robot;

public class Conveyor {
    
    static boolean reversed;
    static CANSparkMax conveyorMotor;
    
  public Conveyor() {
    conveyorMotor = RobotMap.conveyorMotor;
    reversed = false;
    conveyorMotor.setIdleMode(IdleMode.kBrake);
  }
  public void run() {
    reversed = MechanismsJoystick.reverse();

    if (Shooter.upToSpeed) {
      conveyorMotor.set(0.7);
    } else {
      if (MechanismsJoystick.conveyor()) {
        if(!reversed) conveyorMotor.set(0.7);
        if(reversed) conveyorMotor.set(-0.7);
      } else conveyorMotor.set(0);
    }
    

    
  }

  public void autoRun(double startTime, double endTime, double conveyorSpeed) {
    double time = Robot.timer.get();
    if (time > startTime && time < endTime) {
      conveyorMotor.set(conveyorSpeed);
    }
  }
}

