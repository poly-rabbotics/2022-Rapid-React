package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.Controls.MechanismsJoystick;
import frc.robot.Controls.ClimbJoystick;
import frc.robot.Robot;

public class Climb {
		static DoubleSolenoid staticArmPancake, dynamicArmPancake, dynamicArmPivot;
		public static TalonSRX staticArmWinch;
		public static TalonSRX dynamicArmWinch;
		//boolean runAutoClimbHigh, runAutoClimbTraversal;
		static Timer climbTimer;
		static double DAHalfwayPosition, SAHalfWayPosition, DARetractPosition, SARetractPosition;
		public static int autoStep = 0;
		static boolean highBarReached, traversalBarReached;
		static boolean SALimitSwitch, DALimitSwitch;
		static double SAAxis, DAAxis;
		static boolean SARetracted, DARetracted;
		public boolean enableClimb;


		public Climb() {
				staticArmPancake = RobotMap.staticArmPancake;
				dynamicArmPancake = RobotMap.dynamicArmPancake;
				dynamicArmPivot = RobotMap.dynamicArmPivot;
				staticArmWinch = RobotMap.staticArmWinch;
				dynamicArmWinch = RobotMap.dynamicArmWinch;
				//runAutoClimbHigh = false;
				//runAutoClimbTraversal = false;
				climbTimer = new Timer();
				DAHalfwayPosition = 0; //correct(ish) numbers
				DARetractPosition = 0; 
				SAHalfWayPosition = 180000;
				SARetractPosition = 115000;
				dynamicArmWinch.configFactoryDefault();
				staticArmWinch.configFactoryDefault();
				dynamicArmWinch.setSelectedSensorPosition(0);
				staticArmWinch.setSelectedSensorPosition(0);
				dynamicArmWinch.setNeutralMode(NeutralMode.Brake);
				staticArmWinch.setNeutralMode(NeutralMode.Brake);
				enableClimb = false;
		}

		public void run() {
			if (ClimbJoystick.getEnableClimb()) {
				enableClimb = !enableClimb;
			}

			if (MechanismsJoystick.arm()) {
				//autoClimb();
			} else if (enableClimb) { //only run manual climb code if climb is enabled by pressing start on joystick
				dynamicArmWinch.set(ControlMode.PercentOutput, DAAxis);
				staticArmWinch.set(ControlMode.PercentOutput, SAAxis);
				
				if (ClimbJoystick.armPancakeRetract()) {
					dynamicArmPancake.set(Value.kForward); //RETRACT PANCAKES
				}
				if (ClimbJoystick.armPancakeExtend()) {
					dynamicArmPancake.set(Value.kReverse); //EXTEND PANCAKES
				} 
			

				// RUNS DYNAMIC ARM PIVOT CYLINDER
				if (ClimbJoystick.dynamicArmPivot()) {
						if (dynamicArmPivot.get() == Value.kForward) {
								dynamicArmPivot.set(Value.kReverse);
						} else dynamicArmPivot.set(Value.kForward);
				}
				SAAxis = -ClimbJoystick.axis5();
				DAAxis = -ClimbJoystick.axis1();

				if ((SAAxis < 0.15) && (SAAxis > -0.15)) { // JOYSTICK DEADZONE
					SAAxis = 0;
				}
				if ((DAAxis < 0.15) && (DAAxis > -0.15)) { // JOYSTICK DEADZONE
					DAAxis = 0;
				}
				
			}
			//dynamicArmPancake.set(Value.kOff);
				DALimitSwitch = !RobotMap.limitSwitchDA.get();
				SALimitSwitch = !RobotMap.limitSwitchSA.get();
				DARetracted = DALimitSwitch;
				SARetracted = SALimitSwitch;
			SmartDashboard.putNumber("SA Encoder Counts", staticArmWinch.getSelectedSensorPosition());
			SmartDashboard.putNumber("DA Encoder Counts", dynamicArmWinch.getSelectedSensorPosition());
			SmartDashboard.putBoolean("Climb Enabled?", enableClimb);
		}

		public static void autoClimb() { //auto climb step by step
			if (MechanismsJoystick.climbPressed()) autoStep += 1; //increment step every time the button is pressed
			if (autoStep == 9 && !highBarReached) {
				autoStep = 4; //jump back to step 
				highBarReached = true;
			} if (autoStep == 9 && highBarReached) {
				autoStep = 9;
				traversalBarReached = true;
			}
			SmartDashboard.putNumber("Auto Climb Step", autoStep);
			switch (autoStep) { 
				case 1:  //POP UP ARMS
					dynamicArmPancake.set(Value.kForward);
					break;
				case 2:  //PIVOT BACK ARM
					dynamicArmPivot.set(Value.kReverse);
					break;
				case 3:  //FULLY RETRACT STATIC ARM
					fullRetractSA();
					break;
				case 4:  //PIVOT FORWARD ARM TO HIT DYNAMIC ARM
					dynamicArmPivot.set(Value.kForward);
					break;
				case 5:  //RETRACT DYNAMIC ARM FULLY AND EXTEND STATIC ARM SLIGHTLY
					fullRetractDA();
					initPIDControlSA();
					staticArmWinch.set(ControlMode.Position, SAHalfWayPosition);
					break;
				case 6:  //PIVOT TO MAKE STATIC ARM HIT NEXT BAR, FULLY RETRACT STATIC ARM
					dynamicArmPivot.set(Value.kReverse); 
					break;
				case 7: 
					fullRetractSA();
					break;
				case 8:  //FULLY EXTEND DYNAMIC ARM
					initPIDControlDA();
					dynamicArmWinch.set(ControlMode.Position, 0); 
					break; 
				case 9: 

				 
			}
		}

		public static void fullRetractDA() {
			dynamicArmWinch.configFactoryDefault();
			if (!DARetracted) { 
				dynamicArmWinch.set(ControlMode.PercentOutput, -1);
			} else {
				dynamicArmWinch.set(ControlMode.PercentOutput, 0);
			}
		}

		public static void fullRetractSA() {
			staticArmWinch.configFactoryDefault();
			if (!SARetracted) {
				staticArmWinch.set(ControlMode.PercentOutput, -1);
			} else {
				staticArmWinch.set(ControlMode.PercentOutput, 0);
			}
		}

		public static void fullExtendDA() {
			initPIDControlDA();
			dynamicArmWinch.set(ControlMode.Position, 0);
		}


		public static void fullExtendSA() {
			initPIDControlSA();
			staticArmWinch.set(ControlMode.Position, 0);
		}

		public static void initPIDControlSA() {
			staticArmWinch.configFactoryDefault();
			staticArmWinch.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
			staticArmWinch.configNominalOutputForward(0);
			staticArmWinch.configNominalOutputReverse(0);
			staticArmWinch.configPeakOutputForward(1);
			staticArmWinch.configPeakOutputReverse(-1);
			staticArmWinch.setSensorPhase(false);
			staticArmWinch.config_kP(0, .001);
			staticArmWinch.config_kI(0, 0);
			staticArmWinch.config_kD(0, 0);
			staticArmWinch.config_kF(0, 0);
			staticArmWinch.selectProfileSlot(0, 0);
		}

		public static void initPIDControlDA() {
			dynamicArmWinch.configFactoryDefault();
			dynamicArmWinch.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
			dynamicArmWinch.configNominalOutputForward(0);
			dynamicArmWinch.configNominalOutputReverse(0);
			dynamicArmWinch.configPeakOutputForward(1);
			dynamicArmWinch.configPeakOutputReverse(-1);
			dynamicArmWinch.setSensorPhase(false);
			dynamicArmWinch.config_kP(0, .001);
			dynamicArmWinch.config_kI(0, 0);
			dynamicArmWinch.config_kD(0, 0);
			dynamicArmWinch.config_kF(0, 0);
			dynamicArmWinch.selectProfileSlot(0, 0);
		}

		// AUTO CLIMB METHODS (WILL BE CALLED IN TELEOP THOUGH)
		/*************************************************
		WE ARE NOT USING THESE, BUT WE MIGHT SWITCH TO THEM LATER
		**************************************************/
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

			public void methodOne(){
				double time= 302;
				time*=2;
				if (time>5) time=0;
				else time+=2;
				
			}

}
