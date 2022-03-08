package frc.robot.Controls;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;


public class DriveJoystick {
  
  public static XboxController joystick = RobotMap.driveJoystick;
  private static double lastMoveTime = 0;

  public static double getMove(){
    double speed = joystick.getRawAxis(1); //Left joystick on controller, determines forward/backwards movement
    return speed;
  }
  public static double getTurn(){
    double speed = joystick.getRawAxis(4); //Right joystick on controller, determines pivot amount
    return speed;
  }
  public static double axisFive(){
    double axis = joystick.getRawAxis(5); //right joystick vertical
    return axis;
  }
  public static boolean getCameraOrient() {
    return joystick.getRawButtonPressed(2); //B button, switches orientation of f/b controls
  }
  public static boolean getResetEncoder() {
    return joystick.getRawButtonPressed(4); //Y button, zeroes out drive encoders
  }
  public static boolean getStartAutoOrientLeft() {
    return joystick.getRawButtonPressed(5);
  }
  public static boolean getStartAutoOrientRight() {
    return joystick.getRawButtonPressed(6);
  }
  public static boolean getContinueAutoOrient() {
    if (Math.abs(getMove()) > 0.05 || Math.abs(getTurn()) > 0.05) {
      //Robot has moved
      lastMoveTime = Timer.getFPGATimestamp();
    }
    SmartDashboard.putNumber("last move time", lastMoveTime);
    return Timer.getFPGATimestamp() - lastMoveTime > 0.75;
    //return joystick.getRawButton(5) || joystick.getRawButton(6);
  }

  public static boolean getFront(){
    return joystick.getRawButtonPressed(2);
  }

  public static boolean driveMode() {
    return joystick.getRawButtonPressed(8);
  }

  public static boolean getToggleLight(){
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

  // SLOW MODE (unused, maybe useful later)
  public static boolean dPad(){
    if(getPreciseFront() || getPreciseRight() || getPreciseBack() || getPreciseLeft()){
      return true;
    }
    else{
      return false;
    }
  }

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
    return joystick.getRawAxis(3) > 0.6;
  }

  public static boolean runIntake() {
    return joystick.getRawButton(6);
  }

  public static boolean toggleIntakePiston() {
    return joystick.getRawButtonPressed(5);
  }

  public static void rumble(double intensity) {
    joystick.setRumble(RumbleType.kLeftRumble, intensity);
    joystick.setRumble(RumbleType.kRightRumble, intensity);
  }
  
}
