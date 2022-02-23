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
    double DAHalfwayPosition;

    public Climb() {
        staticArmPancake = RobotMap.staticArmPancake;
        dynamicArmPancake = RobotMap.dynamicArmPancake;
        dynamicArmPivot = RobotMap.dynamicArmPivot;
        staticArmWinch = RobotMap.staticArmWinch;
        dynamicArmWinch = RobotMap.dynamicArmWinch;
        runAutoClimbHigh = false;
        runAutoClimbTraversal = false;
        climbTimer = new Timer();
        DAHalfwayPosition = 5000; //F I N D   O U T   T H I S   N U M B E R
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
        if (runAutoClimbHigh) { //AT THIS POINT, STATIC ARM WILL ALREADY BE HOOKED ON TO MIDDLE BAR
          climbTimer.start();
          autoDAPivot(0, 2, "back"); //pivots DA back
          autoDAPancake(2, 5); //releases DA to extend
          autoSAWinch(0, 7, 0); //pulls robot up to middle bar
          //end position 0 because that would retract it after it extended to begin with, pulling up robot
          autoDAPivot(7, 11, "up"); //pivots DA to upright position, applying pressure to high bar
          autoDAWinch(8, 13, DAHalfwayPosition); //pulls up robot slightly to get off of mid bar, delays bc swinging
          autoDAWinch(18, 25, 0); //pulls up robot to high bar completely
          //again end position of zero to return to original position
        }
        
        else if (runAutoClimbTraversal) { //AT THIS POINT, STATIC ARM WILL ALREADY BE HOOKED ON TO MIDDLE BAR
          climbTimer.start();
          autoDAPivot(0, 2, "back"); //pivots DA back
          autoDAPancake(2, 5); //releases DA to extend
          autoSAWinch(0, 7, 0); //pulls robot up to middle bar
          //end position 0 because that would retract it after it extended to begin with, pulling up robot
          autoDAPivot(7, 12, "up"); //pivots DA to upright position, applying pressure to high bar
          autoDAWinch(8, 13, DAHalfwayPosition); //pulls up robot slightly to get off mid bar, delays bc swinging
          autoDAWinch(18, 25, 0);
          //again end position of zero to return to original position
          //REACHES HIGH BAR HERE

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

      public static void autoDAPivot(double startTime, double endTime, String position) {
        double time = climbTimer.get();
        if (time > startTime && time < endTime) {
            if (position == "up") {
              dynamicArmPivot.set(Value.kReverse);
            }
            else if (position == "back") {
              dynamicArmPivot.set(Value.kForward);
            }
            
        }
      }

      public static void runAutoClimbHigh() {

      }
}
