package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.RobotMap;
import frc.robot.Controls.MechanismsJoystick;
import frc.robot.Robot;

public class Climb {
    static DoubleSolenoid staticArmPancake, dynamicArmPancake, dynamicArmPivot;
    static TalonSRX staticArmWinch, dynamicArmWinch;

    public Climb() {
        staticArmPancake = RobotMap.staticArmPancake;
        dynamicArmPancake = RobotMap.dynamicArmPancake;
        dynamicArmPivot = RobotMap.dynamicArmPivot;
        staticArmWinch = RobotMap.staticArmWinch;
        dynamicArmWinch = RobotMap.dynamicArmWinch;
    }

    public void run() {
        /*
        if (MechanismsJoystick.staticArmPancakeRelease()) {
            staticArmPancake.set(Value.kReverse);
        }
        if (MechanismsJoystick.dynamicArmPancakeRelease()) {
            dynamicArmPancake.set(Value.kReverse);
        }
        */

        if (MechanismsJoystick.dynamicArmPivot()) {
            if (dynamicArmPivot.get() == Value.kForward) {
                dynamicArmPivot.set(Value.kReverse);
            } else dynamicArmPivot.set(Value.kForward);
        }

        /*
        if (MechanismsJoystick.staticArmRun()) {
            staticArmWinch.set(ControlMode.PercentOutput, 0.1);
        } else staticArmWinch.set(ControlMode.PercentOutput, 0);

        if (MechanismsJoystick.dynamicArmRun()) {
            dynamicArmWinch.set(ControlMode.PercentOutput, 0.1);
        } else dynamicArmWinch.set(ControlMode.PercentOutput, 0);
        */

        //dynamicArmWinch.set(ControlMode.PercentOutput, MechanismsJoystick.testJoystick());
    }

    
    public static void autoDAWinch(double startTime, double endTime, double endPosition) {
        double time = Robot.timer.get();
        if (time > startTime && time < endTime) {
          dynamicArmWinch.set(ControlMode.Position, endPosition);
        }
      }

      public static void autoSAWinch(double startTime, double endTime, double endPosition) {
        double time = Robot.timer.get();
        if (time > startTime && time < endTime) {
          dynamicArmWinch.set(ControlMode.Position, endPosition);
        }
      }

      public static void autoDAPancake(double startTime, double endTime) {
        double time = Robot.timer.get();
        if (time > startTime && time < endTime) {
            dynamicArmPancake.set(Value.kReverse);
        }
      }

      public static void autoSAPancake(double startTime, double endTime) {
        double time = Robot.timer.get();
        if (time > startTime && time < endTime) {
            staticArmPancake.set(Value.kReverse);
        }
      }

      public static void autoDAPivot(double startTime, double endTime) {
        double time = Robot.timer.get();
        if (time > startTime && time < endTime) {
            dynamicArmPivot.set(Value.kReverse);
        }
      }
}