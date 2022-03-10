// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.revrobotics.CANSparkMax;

import frc.robot.RobotMap;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Controls.MechanismsJoystick;
import frc.robot.Robot;

public class Conveyor {
    
    static boolean reversed;
    static CANSparkMax conveyorMotor;
    static double conveyorSpeed;
    double setpoint = 0.7;
    boolean ballDetect;
    Timer ballSpacer = new Timer();
    boolean ballDetectedLow, ballDetectedHigh;
    public int ballCount;
    
  public Conveyor() {
    conveyorMotor = RobotMap.conveyorMotor;
    reversed = false;
    conveyorMotor.setIdleMode(IdleMode.kBrake);
    ballCount = 0;
  }
  public void run() {
    ballDetectedLow = !RobotMap.proxSensorLow.get();
    ballDetectedHigh = !RobotMap.proxSensorHigh.get();
    
    /*
    if (Shooter.upToSpeed) {
      conveyorSpeed = 0.7;
    } else if (!Shooter.upToSpeed){ */
      if (MechanismsJoystick.conveyor()&&!ballDetectedHigh) {
        conveyorSpeed = 0.7;

      } else if (MechanismsJoystick.conveyor2()){ 
        conveyorSpeed = -0.7;
      } else if (ballDetectedLow && !ballDetectedHigh) {
      conveyorSpeed = setpoint;
      ballDetect = true;
      ballSpacer.reset();
      ballSpacer.start();
      ballCount = 1;
    } else if (!ballDetectedLow && ballDetect) {
      SmartDashboard.putNumber("Ball Spacer", ballSpacer.get());
      conveyorSpeed = setpoint;
      if (ballSpacer.get() > 0.2) {
        ballDetect = false;
        conveyorSpeed = 0;
      }
    } else if(!MechanismsJoystick.farShot() && !MechanismsJoystick.closeShot()) conveyorSpeed = 0;
    
    if (ballDetectedHigh && ballDetectedLow) ballCount = 2;
    else ballCount = 0;

    conveyorMotor.set(conveyorSpeed);
    
  }

  public void autoRun(double startTime, double endTime, double conveyorSpeed) {
    double time = Robot.autoTimer.get();
    if (time > startTime && time < endTime) {
      conveyorMotor.set(conveyorSpeed);
    }
  }
}

