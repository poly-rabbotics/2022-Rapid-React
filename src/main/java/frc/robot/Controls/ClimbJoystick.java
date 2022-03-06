package frc.robot.Controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

public class ClimbJoystick {
    private static Joystick joystick = RobotMap.climbJoystick;

    public static boolean armPancakeRetract() {
        
        if(joystick.getPOV() == 180){
          return true;
        }
        else{
          return false;
        }
        
        
      }

    public static boolean armPancakeExtend() {
      
        if(joystick.getPOV() == 0){
          return true;
        }
        else{
          return false;
        }
      }

    public static boolean dynamicArmPivot() {
        return joystick.getRawButtonPressed(1);
      }

      public static double axis1() {
        return joystick.getRawAxis(1);
      }     
      public static double axis5() {
        return joystick.getRawAxis(5);
      } 
}
