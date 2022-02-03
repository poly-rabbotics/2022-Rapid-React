package frc.robot.Controls;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotMap;

public class MechanismsJoystick {
    private static Joystick joystick = RobotMap.mechanismsJoystick;


    public static boolean startGalacticSearch(){
    return  joystick.getRawButton(4); //needs to be given an actual number
      }

     public static boolean farShot(){
        return  joystick.getRawButton(7); //needs to be given an actual number
          }

      public static boolean midShot(){
          return  joystick.getRawButton(6); //needs to be given an actual number
        }
        public static boolean closeShot(){
          return  joystick.getRawButton(5); //needs to be given an actual number
        }

        public static boolean conveyor(){
          return joystick.getRawButton(8);
        }

        public static boolean reverse(){
          return joystick.getRawButton(11);
        }

        public static boolean intake(){
          return joystick.getRawButton(9);
        }
        
        public static boolean climbPressed(){
          return joystick.getRawButtonPressed(10);
        }
        public static boolean climb(){
          return joystick.getRawButton(10);
        }
        public static boolean arm(){
          return joystick.getRawButton(4);
        }
        
        
        public static boolean autoSwitchOne(){
          return joystick.getRawButton(1);
        }
    
        public static boolean autoSwitchTwo(){
          return joystick.getRawButton(2);
        }
    
        public static boolean autoSwitchThree(){
          return joystick.getRawButton(3);
        }

        public static boolean shooterSpeedIncrease(){ 
          return joystick.getRawButtonPressed(1); //A button
        }
      
        public static boolean shooterSpeedDecrease(){ 
          return joystick.getRawButtonPressed(3); //B button
        }
      
        public static boolean shooterButton(){
          return joystick.getRawButton(5);
        }

        public static boolean intakeButton(){
          return joystick.getRawButton(6);
        }

        public static boolean conveyorReverse() {
          return joystick.getRawButtonPressed(3);
        }

        public static boolean runConveyor() {
          return joystick.getRawButton(1);
        }
        
        public static boolean shooterActive() {
          return joystick.getRawAxis(3) > 0.6;
        }

        public static boolean staticArmPancakeRelease() {
          if(joystick.getPOV() == 0){
            return true;
          }
          else{
            return false;
          }
        }

        public static boolean dynamicArmPancakeRelease() {
          if(joystick.getPOV() == 180){
            return true;
          }
          else{
            return false;
          }
        }

        public static boolean staticArmRun() {
          if(joystick.getPOV() == 90){
            return true;
          }
          else{
            return false;
          }
        }

        public static boolean dynamicArmRun() {
          if(joystick.getPOV() == 270){
            return true;
          }
          else{
            return false;
          }
        }

        public static boolean dynamicArmPivot() {
          return joystick.getRawAxis(2) > 0.6;
        }

        public static boolean targetHub() {
          return joystick.getRawButton(11);
        }

        public static boolean blue() {
          return joystick.getRawButton(12);
        }   
        public static boolean red() {
          return !joystick.getRawButton(12);
        }           

}