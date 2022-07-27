package frc.robot.Controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

//This is a class O_O woahhhhh. We named it ClimbJoystick because you CLIMB with the JOYSTICK *Mind explodes*.
public class ClimbJoystick {
    //This is a Joystick variable that we imported above. We difined 'joystick' as a Joystick... cuz yeah.
    private static Joystick joystick = RobotMap.climbJoystick;

    //This is a boolean that we named armPancakeRetract. We made it so it Retracts the arm Pancake. Ik, its insane.
    public static boolean armPancakeRetract() {
        
        //This checks if the POV for the joystick is 180 degrees.
        if(joystick.getPOV() == 180){
          //This returns it as true.
          return true;
        }
        //This checks if the POV is not 180 degrees.
        else{
          //This returns it as true.
          return false;
        }
        
        
      }
    //This boolean is named armPancakeExtend. Unexpectedly, this method Extends the arm Pancake.
    public static boolean armPancakeExtend() {
        //This checks if the POV of the arm is 0 degrees (Aka: d/dx[69])
        if(joystick.getPOV() == 0){
          //Lol. So true.
          return true;
        }
        //If the POV is not 0 degress this runs.
        else{
          //Is Rohan big brain?
          return false;
        }
      }
    
    //This is a boolean called dynamicArmPivot. It makes the dynamic arm Pivot! ooga booga XD poggy woggy. I'm going insane.
    public static boolean dynamicArmPivot() {
      //This makes it so the button: '1' activates the dynamic arm pivot
                                      return joystick.getRawButtonPressed(1);
    }
    
    //The following doubles do the same as the method above.
      public static double axis1() {
        return joystick.getRawAxis(1);
      }     
      public static double axis5() {
        return joystick.getRawAxis(5);
      } 

      public static boolean getEnableClimb() {
        return joystick.getRawButtonPressed(8);
      }
}
