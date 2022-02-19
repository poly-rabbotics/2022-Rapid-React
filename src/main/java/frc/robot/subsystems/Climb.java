package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.RobotMap;
import frc.robot.Controls.MechanismsJoystick;
import frc.robot.Robot;

public class Climb {
    static DoubleSolenoid staticArmPancake, dynamicArmPancake, dynamicArmPivot;
    static TalonSRX staticArmWinch, dynamicArmWinch;
    boolean runAutoClimbHigh, runAutoClimbTraversal;
    static Timer climbTimer;

    public Climb() {
        staticArmPancake = RobotMap.staticArmPancake;
        dynamicArmPancake = RobotMap.dynamicArmPancake;
        dynamicArmPivot = RobotMap.dynamicArmPivot;
        staticArmWinch = RobotMap.staticArmWinch;
        dynamicArmWinch = RobotMap.dynamicArmWinch;
        runAutoClimbHigh = false;
        runAutoClimbTraversal = false;
        climbTimer = new Timer();
    }

    public void run() {
        /* PANCAKE PIN RELEASE, SENDS UP ARMS
        if (MechanismsJoystick.staticArmPancakeRelease()) {
            staticArmPancake.set(Value.kReverse);
        }
        if (MechanismsJoystick.dynamicArmPancakeRelease()) {
            dynamicArmPancake.set(Value.kReverse);
        }
        */

        // RUNS DYNAMIC ARM PIVOT CYLINDER
        if (MechanismsJoystick.dynamicArmPivot()) {
            if (dynamicArmPivot.get() == Value.kForward) {
                dynamicArmPivot.set(Value.kReverse);
            } else dynamicArmPivot.set(Value.kForward);
        }

        /* RUNS STATIC AND DYNAMIC ARM MOTORS
        if (MechanismsJoystick.staticArmRun()) {
            staticArmWinch.set(ControlMode.PercentOutput, 0.1);
        } else staticArmWinch.set(ControlMode.PercentOutput, 0);

        if (MechanismsJoystick.dynamicArmRun()) {
            dynamicArmWinch.set(ControlMode.PercentOutput, 0.1);
        } else dynamicArmWinch.set(ControlMode.PercentOutput, 0);
        */

        //dynamicArmWinch.set(ControlMode.PercentOutput, MechanismsJoystick.testJoystick());
        if (runAutoClimbHigh) {
          climbTimer.start();
          
        }
    }

    // AUTO CLIMB METHODS (WILL BE CALLED IN TELEOP THOUGH)
    public static void autoDAWinch(double startTime, double endTime, double endPosition) {
        double time = climbTimer.get();
        if (time > startTime && time < endTime) {
          dynamicArmWinch.set(ControlMode.Position, endPosition);
        }
      }

      public static void autoSAWinch(double startTime, double endTime, double endPosition) {
        double time = climbTimer.get();
        if (time > startTime && time < endTime) {
          dynamicArmWinch.set(ControlMode.Position, endPosition);
        }
      }

      public static void autoDAPancake(double startTime, double endTime) {
        double time = climbTimer.get();
        if (time > startTime && time < endTime) {
            dynamicArmPancake.set(Value.kReverse);
        }
      }

      public static void autoSAPancake(double startTime, double endTime) {
        double time = climbTimer.get();
        if (time > startTime && time < endTime) {
            staticArmPancake.set(Value.kReverse);
        }
      }

      public static void autoDAPivot(double startTime, double endTime) {
        double time = climbTimer.get();
        if (time > startTime && time < endTime) {
            dynamicArmPivot.set(Value.kReverse);
        }
      }

      public static void runAutoClimbHigh() {

      }
}
