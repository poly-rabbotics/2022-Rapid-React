package frc.robot.Controls;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;


public class DriveJoystick {
  
  //Sets the drive joystick to the XboxController that was imported above
  public static XboxController joystick = RobotMap.driveJoystick;
  private static double lastMoveTime = 0;

  public static double getMove(){
    //Sets the speed of the joystick equal to the axis of input 1 on the Xbox Controller
    double speed = joystick.getRawAxis(1); //Left joystick on controller, determines forward/backwards movement
    return speed;
  }
  public static double getTurn(){
    //Sets the speed of the joystick equal to the axis of input 4 on the Xbox Controller
    double speed = joystick.getRawAxis(4); //Right joystick on controller, determines pivot amount
    return speed;
  }
  public static double axisFive(){
    //Sets the axis double equal to input 5
    double axis = joystick.getRawAxis(5); //right joystick vertical
    return axis;
  }
  public static boolean getCameraOrient() {
    //Checks if input 2 on the controller is pressed and held.
    return joystick.getRawButtonPressed(2); //B button, switches orientation of f/b controls
  }
  /*
  public static boolean getResetEncoder() {
    return joystick.getRawButtonPressed(4); //UNUSED
  }
  */
  public static boolean getStartAutoOrientLeft() {
    return joystick.getRawButtonPressed(5);
  }
  public static boolean getStartAutoOrientRight() {
    return joystick.getRawButtonPressed(6);
  }
  public static boolean getContinueAutoOrient() {
    //Checks if the movement of the robot or the turning of the robot is greater than 0.05
    if (Math.abs(getMove()) > 0.05 || Math.abs(getTurn()) > 0.05) {
      //Robot has moved
      //Sets the variable "lastMoveTime" as a timer that starts when the robot last moved.
      lastMoveTime = Timer.getFPGATimestamp();
    }
    //Displays a statistic on SmartDashboard called "last move time" that shows the time since the robot last moved
    SmartDashboard.putNumber("last move time", lastMoveTime);
    //Returns the timer minus 0.75 seconds
    return Timer.getFPGATimestamp() - lastMoveTime > 0.75;
    //return joystick.getRawButton(5) || joystick.getRawButton(6);
  }

  //More buttons...
  public static boolean getFront(){
    return joystick.getRawButtonPressed(2);
  }

  public static boolean driveMode() {
    return joystick.getRawButtonPressed(8);
  }

  public static boolean getToggleLight(){
    //Checks if input 3 is pressed. If it is pressed then the method returns true for one moment.
    return joystick.getRawButton(3);
  }

  public static boolean getToggleGears(){
    return joystick.getRawButtonPressed(3); //X button
  }
  public static boolean getToggleDriveMode(){
    return joystick.getRawButtonPressed(8); //start button
  }
  public static boolean aim(){ //activates limelight auto-aim
    return joystick.getRawButton(1); //A button
  }

  // SLOW MODE 
  public static boolean dPad(){
    //If any of the following booleans are true then the output of this method is true.
    if(getPreciseFront() || getPreciseRight() || getPreciseBack() || getPreciseLeft()){
      return true;
    }
    else{
      return false;
    }
  }

  //The following methods check when the POV of the joystick is 0, 90, 180 or 270
  public static boolean getPreciseFront(){
    if(joystick.getPOV() == 0){
      return true;
    }
    else{
      return false;
    }
  }
  public static boolean getPreciseRight(){
    if(joystick.getPOV() == 90){
      return true;
    }
    else{
      return false;
    }
  }
  public static boolean getPreciseBack(){
    if(joystick.getPOV() == 180){
      return true;
    }
    else{
      return false;
    }
  }
  public static boolean getPreciseLeft(){
    if(joystick.getPOV() == 270){
      return true;
    }
    else{
      return false;
    }
  }
   
  public static boolean getDriveAllowClimb(){
    return joystick.getRawButton(8);
  }

  public static boolean getDriveToggleClimb(){
    return joystick.getRawButtonPressed(7);
  }

  public static boolean enablePIDMode() {
    //Checks if the axis of input 3 is greater than 0.6
    return joystick.getRawAxis(3) > 0.6;
  }

  public static boolean runIntake() {
    return joystick.getRawButton(6);
  }

  public static boolean runIntakeReverse() {
    return joystick.getRawButton(5);
  }
  public static boolean toggleIntakePiston() {
    return joystick.getRawAxis(2) > 0.7;
  }

  //RUMBLE RUMBLE lol
  public static void rumble(double intensity) {
    joystick.setRumble(RumbleType.kLeftRumble, intensity);
    joystick.setRumble(RumbleType.kRightRumble, intensity);
  }
  
}
