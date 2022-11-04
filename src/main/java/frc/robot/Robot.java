// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.TimeUnit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Controls.DriveJoystick;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static Intake intake;
  	public static Shooter shooter;
  	public static Conveyor conveyor;
  	public static Climb climb;
  	public static Drive drive;
  	public static LEDLights LEDs;
  	public static Timer masterTimer, autoTimer;
  	public static AHRSGyro gyro;
  	public static double leftEncoderCounts, rightEncoderCounts;
  	public static LIDAR lidar;
  	boolean pressureGood;
  	boolean isGyroReset = false;
  	Compressor comp;
  	PneumaticHub hub;
  	AutoModes auto;
  	DashboardLog dashboardLog;
  	private double prevPressure = 0.0;
  
  	@Override
  	public void robotInit() {
    	RobotMap.initJoysticks();

    	comp = new Compressor(1, PneumaticsModuleType.REVPH);
    
    	RobotMap.initShooter();
    	shooter = new Shooter();
    	RobotMap.initIntake();
    	intake = new Intake();
    
    	RobotMap.initDriveMotors();  
    	RobotMap.initDrivePancakes();
    	drive = new Drive();
    
    	RobotMap.initClimb();
    	climb = new Climb();
    
    	LEDs = new LEDLights();
    
    	masterTimer = new Timer();
    	//masterTimer.start();
    	autoTimer = new Timer();
    
    
    	gyro = new AHRSGyro();
    
    	RobotMap.initConveyor();
    	conveyor = new Conveyor();
    
    	lidar = new LIDAR();
    	lidar.start();
    
    	comp.enableDigital();
    	auto = new AutoModes();
    	RobotMap.initLimitSwitches();
    	RobotMap.initProxSensors();
    	RobotMap.initPressureTransducer();
    	RobotMap.initLimelight();
    
    	// Starts the limelight service and calls the limelights run() methods at a fixed rate of once every 10 ms or at 100hz.
    	RobotMap.limelightService.scheduleAtFixedRate(RobotMap.limelight, 0, 10, TimeUnit.MILLISECONDS);

    	dashboardLog = new DashboardLog();
  	}
  
  	/**
	 * This function is called every robot packet, no matter the mode. Use this for items like
	 * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   	 *
   	 * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   	 * SmartDashboard integrated updating.
   	 */
  	@Override
 	public void robotPeriodic() {
    	SmartDashboard.putNumber("Timer", masterTimer.get());
    	SmartDashboard.putBoolean("DA Limit Switch", !RobotMap.limitSwitchDA.get());
    	SmartDashboard.putBoolean("SA Limit Switch", !RobotMap.limitSwitchSA.get());

    	SmartDashboard.putNumber("Gyro Degrees", gyro.getDegrees());

    	SmartDashboard.putNumber("limelight servo", Limelight.servo.get());

    
    	leftEncoderCounts = Drive.leftBack.getSelectedSensorPosition();
    	rightEncoderCounts = -1 * Drive.rightBack.getSelectedSensorPosition();
    	SmartDashboard.putNumber("left Encoder Feet", leftEncoderCounts / 13445);
    	SmartDashboard.putNumber("right Encoder Feet", rightEncoderCounts / 13445);
    	SmartDashboard.putNumber("left Encoder Counts", leftEncoderCounts);
    	SmartDashboard.putNumber("right Encoder Counts", rightEncoderCounts);
    	SmartDashboard.putNumber("left Encoder Degrees", leftEncoderCounts / Drive.ENCODER_COUNTS_PER_360 / 360);
    	SmartDashboard.putNumber("right Encoder Degrees", rightEncoderCounts / Drive.ENCODER_COUNTS_PER_360 / 360);
    	SmartDashboard.putBoolean("prox 1", RobotMap.proxSensorLow.get());
    	SmartDashboard.putBoolean("prox 2", RobotMap.proxSensorHigh.get());

    	double robotPressure = 40.16 * (RobotMap.pressureTransducer.getVoltage() - 0.52);
    
    	SmartDashboard.putNumber("Robot Pressure", robotPressure);

	    // this is an aproximation based on the 50hz clock that periodic should run on.
    	// in the case that a cycle takes longer than 20 millisecond, or is run faster than
    	// 50 hz for whatever reason, this value will be innacurate.
		SmartDashboard.putNumber("Pressure per Iteration Lost", prevPressure - robotPressure);
    
    	prevPressure = robotPressure;

    	pressureGood = robotPressure > 60;
    	SmartDashboard.putBoolean("Pressure Good?", pressureGood);  
    
    	if(isDisabled()) 
			LEDs.run(1);

    	SmartDashboard.putNumber("LIDAR Distance Inches", lidar.getDistance() / 2.54);
    	//CLIMB DATA
    	SmartDashboard.putNumber("DA encoder counts", Climb.dynamicArmWinch.getSelectedSensorPosition());
    	SmartDashboard.putNumber("SA encoder counts", Climb.staticArmWinch.getSelectedSensorPosition());
    	/*
    	if (masterTimer.get() > 5 && !isGyroReset) {
      		gyro.reset();
      		isGyroReset = true;
    	} */
    
    	if (masterTimer.get() > 110 && masterTimer.get() < 111) {
     		DriveJoystick.rumble(0.1);
    	} else {
      		DriveJoystick.rumble(0);
    	} 

		auto.setAutoMode();
		
    	SmartDashboard.putBoolean("Auto movement completed?", drive.movementCompleted);
    	SmartDashboard.putBoolean("Auto turn completed?", drive.turnCompleted);

    	SmartDashboard.putBoolean("Climb Enabled?", climb.enableClimb);

    	if (DriveJoystick.resetGyroButton()) 
			gyro.reset();
  	}

  	/**
   	 * This autonomous (along with the chooser code above) shows how to select between different
   	 * autonomous modes using the dashboard. The sendable chooser code works with the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   	 * uncomment the getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to the switch structure
     * below with additional strings. If using the SendableChooser make sure to add them to the
     * chooser code above as well.
     */
	@Override
	public void autonomousInit() {
    	//gyro.reset();
    	masterTimer.start();
    	autoTimer.reset(); 
    	autoTimer.start(); 
     
    	drive.initAutoDrive();
   		// drive.initPercentOutputDrive();
    	drive.resetEncoders(); 
  	}

  	/** This function is called periodically during autonomous. */
  	@Override
  	public void autonomousPeriodic() {
    	auto.runAuto();
    
    	/*
    	RobotMap.drivePancake.set(Value.kForward);
    	shooter.autoRun(0, 15, -4600);
   		conveyor.autoRun(1, 4, 0.7);
    	//conveyor.autoRun(4, 5, 0);
    	intake.deployIntake(2, 5, true);
    	drive.moveByInches(3, 5, 60); //fix this distance setpoint for actual field geometry
    	intake.autoRun(3, 6, 0.5);
    	drive.moveByInches(6, 8, -5);
    	conveyor.autoRun(4, 10, 0.2);
    	conveyor.autoRun(10, 15, 0.7);
    	intake.autoRun(6, 15, 0);
    	LEDs.up(2); */
    	/*
    	drive.turnByDegrees(10, 12, 180);
    	drive.moveByInches(12, 15, -60);
    	*/
	}

  	/** This function is called once when teleop is enabled. */
 	@Override
  	public void teleopInit() {
    	//gyro.reset();
    	Climb.dynamicArmWinch.setSelectedSensorPosition(0);
    	Climb.staticArmWinch.setSelectedSensorPosition(0);
    	Climb.autoStep = 0;
    	drive.resetEncoders();
		
  	}

  	/** This function is called periodically during operator control. */
  	@Override
  	public void teleopPeriodic() {
    	try {
      		shooter.run(); 
    	} 
    	catch (Exception e) {
      		dashboardLog.logError(e);
    	}
    
		try {
     	 	conveyor.run(); 
    	} 
    	catch (Exception e) {
      		dashboardLog.logError(e);
    	}
    
		try {
      		intake.run(); 
    	} 
    	catch (Exception e) {
      		dashboardLog.logError(e);
    	}
    
		try {
      		climb.run();    
    	} 
    	catch (Exception e) {
      		dashboardLog.logError(e);
    	}
    
		try {
    	  	drive.run();    
    	} 
    	catch (Exception e) {
      		dashboardLog.logError(e);
    	}

    	/*
    	if(MechanismsJoystick.dynamicArmPancakeRelease()) { 
      		RobotMap.testSolenoidOne.set(Value.kOff);
      		RobotMap.testSolenoidTwo.set(Value.kOff);
      		RobotMap.testSolenoidThree.set(Value.kReverse);
	    } else if (MechanismsJoystick.staticArmPancakeRelease()) {
      		RobotMap.testSolenoidOne.set(Value.kOff);
	      	RobotMap.testSolenoidTwo.set(Value.kOff);
      		RobotMap.testSolenoidThree.set(Value.kForward);
    	}
    	*/

    	//LEDS
    	//Master Timer (110 Seconds)
    	if(masterTimer.get() > 110 && masterTimer.get() < 112) {
			LEDs.run(12);
    	//Master Timer (All times other than 110 seconds)
		} else {
      		//Ball Count > 0
      		if (climb.enableClimb) { 
				LEDs.run(10);
			} else if(conveyor.ballCount > 0 && !climb.enableClimb) { 
				LEDs.run(4); //If we have 2 balls use mode 4
      		} else if(conveyor.ballCount > 0 && climb.enableClimb) {
				LEDs.run(10); //If we have 2 balls and are climbing use mode 10
			} else if(conveyor.ballCount == 0 && !climb.enableClimb) {
				//If we are < 2 balls and arent climbing ...
        		if(Drive.PIDDriveActive) {                                           
					//If PID active ...
          			if(shooter.shooterRunning) 
						LEDs.run(8); //If shooter running in PID use mode 8
          			else if(!shooter.shooterRunning) {
						//If shooter is not running ...
            			if(drive.highTorqueModeActive) 
							LEDs.run(6); //If HTM is active in PID use mode 6
            			else if(!drive.highTorqueModeActive) 
							LEDs.run(9); //If HTM is inactive in PID use mode 9
          			}
        		}
        		else if(!Drive.PIDDriveActive) {                                  
          			if(drive.highTorqueModeActive) {
						LEDs.run(7); //If HTM is active: no PID use mode 7
					} else if(!drive.highTorqueModeActive) {
						//If HTM is inactive: no PID ...
            			if(!shooter.shooterRunning) 
							LEDs.run(2); //If shooter is not running use mode 2
            			else if(shooter.shooterRunning) 
							LEDs.run(5); //If shooter is running use mode 5
          			}
				}
        	}
    	}
  	}

    /*
    if(masterTimer.get() > 110 && masterTimer.get() < 112) LEDs.run(12);
    else if(shooter.shooterRunning && !Drive.PIDDriveActive && !drive.highTorqueModeActive) LEDs.run(5); SHOOTER NO PID NO HTM
    else if(Drive.PIDDriveActive && !drive.highTorqueModeActive) LEDs.run(9);
    else if(drive.highTorqueModeActive && !Drive.PIDDriveActive) LEDs.run(7);
    else if(Drive.PIDDriveAc
	tive && shooter.shooterRunning) LEDs.run(8);          SHOOTER PID
    else if(Drive.PIDDriveActive && drive.highTorqueModeActive) LEDs.run(6);
    else if(conveyor.ballCount > 0) LEDs.run(4);
    else if(MechanismsJoystick.arm()) LEDs.run(10);
    else LEDs.run(2);
    */
  

  	/** This function is called once when the robot is disabled. */
  	@Override
  	public void disabledInit() {}

  	/** This function is called periodically when disabled. */
  	@Override
  	public void disabledPeriodic() { }

  	/** This function is called once when test mode is enabled. */
  	@Override
  	public void testInit() {
  	}

  	/** This function is called periodically during test mode. */
  	@Override
  	public void testPeriodic() {
  	}
}
