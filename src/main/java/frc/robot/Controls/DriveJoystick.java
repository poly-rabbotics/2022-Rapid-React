/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Controls;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DriveJoystick {
  
  public static Joystick joystick = RobotMap.driveJoystick;
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

  public static boolean getXButton(){
    return joystick.getRawButton(3);
  }
  public static boolean getYButton(){
    return joystick.getRawButton(4);
  }
  public static boolean aim(){ //activates limelight auto-aim
    return joystick.getRawButton(1); 
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


}
