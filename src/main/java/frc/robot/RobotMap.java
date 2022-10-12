package frc.robot;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.helperClasses.LightRenderer;

import java.util.concurrent.*;
import edu.wpi.first.wpilibj.*;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class RobotMap {
	// NOTE: this should be the only instance of this class, ever.
	// It only works if you use the same one everywhere.
	public static DashboardLog dashboardLog = new DashboardLog();


	/*
	 * Feilds are sorted by and titled according to the method in
	 * which they are initialized.
	 */

	/* --> LED Lights <-- */
	// Using 100 for buffer length, could easily be changed, was chosen arbatrarily.
	
	//public static final AddressableLED led = new AddressableLED(1);
	
	/* --> Joysticks <-- */
    public static XboxController driveJoystick;
    public static Joystick mechanismsJoystick;
    public static Joystick climbJoystick;
    public static Joystick guitarJoystick;

	/* --> Drive Motors <-- */
    public static TalonSRX leftBack;
    public static TalonSRX leftFront;
    public static TalonSRX rightBack;
    public static TalonSRX rightFront;

	/* --> Drive Pancakes <-- */
    public static DoubleSolenoid drivePancake;
    
	/* --> Intake <-- */
	public static CANSparkMax intakeMotor;
    public static DoubleSolenoid intakeSolenoid;
   
	/* --> Conveyor <-- */
	public static CANSparkMax conveyorMotor;
    
	/* --> Shooter <-- */
	public static TalonSRX shooterMotor;

	/* --> Climb <-- */
    public static TalonSRX staticArmWinch;
    public static TalonSRX dynamicArmWinch;
    public static DoubleSolenoid dynamicArmPivot;
    public static DoubleSolenoid dynamicArmPancake;
    public static DoubleSolenoid staticArmPancake;
    
	/* --> Limit Switches <-- */
    public static DigitalInput limitSwitchDA;
    public static DigitalInput limitSwitchSA;

	/* --> Prox Sensors <-- */
    public static DigitalInput proxSensorLow; 
    public static DigitalInput proxSensorHigh;
	
	/* --> Lime Light <-- */
    public static Servo limelightServo;
    public static Limelight limelight;
	public static ScheduledExecutorService limelightService;

	/* --> Pressure Transducer <-- */
	public static AnalogInput pressureTransducer;

	public static void initLEDLights() {
		try {
			//lightRendererService = Executors.newSingleThreadScheduledExecutor();
		} catch (Exception e) {
			dashboardLog.logError("Error occured while initializing LED lights");
			dashboardLog.logError(e);
		}
	}

	public static void initJoysticks(){
        try{
            driveJoystick = new XboxController(0);
            mechanismsJoystick = new Joystick(1);
            climbJoystick = new Joystick(2);
            guitarJoystick = new Joystick(3); 
        } catch (Exception e) {
        	dashboardLog.logError("Error occured while initializing joysticks.");
			dashboardLog.logError(e);
		} 
    }
     
    public static void initDriveMotors() {
        try{
            leftBack = new TalonSRX(1);
            leftFront = new TalonSRX(2); 
            rightBack = new TalonSRX(3);
            rightFront = new TalonSRX(4);
        } catch (Exception e) {
        	dashboardLog.logError("Error occured while initializing drive motors.");
			dashboardLog.logError(e);
		}
    }

    public static void initDrivePancakes() {
        try {
	        drivePancake = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
        } catch (Exception e) {
        	dashboardLog.logError("Error occured while initializing drive pancakes.");
			dashboardLog.logError(e);
		}
    }
    
    public static void initIntake() {
        try {
            intakeMotor = new CANSparkMax(5, MotorType.kBrushless);
            intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);
        } catch (Exception e) {
        	dashboardLog.logError("Error occured while initializing intake.");
			dashboardLog.logError(e);
		}
    }

    public static void initConveyor() {
        try {
            conveyorMotor = new CANSparkMax(6, MotorType.kBrushless);
        } catch (Exception e) {
    		dashboardLog.logError("Error occured while initializing conveyor.");
			dashboardLog.logError(e);
		}

    }

    public static void initShooter() {
        try {
            shooterMotor = new TalonSRX(7);
        } catch (Exception e) {
        	dashboardLog.logError("Error occured while initializing shooter.");
			dashboardLog.logError(e);
		}
    }
    
    public static void initClimb() {
        try {
            staticArmWinch = new TalonSRX(5);
            dynamicArmWinch = new TalonSRX(6);
            dynamicArmPivot = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);
            dynamicArmPancake = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);
            staticArmPancake = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8, 9);
        } catch (Exception e) {
        	dashboardLog.logError("Error occured while initializing climb.");
			dashboardLog.logError(e);
		}
    } 
	
	public static void initLimitSwitches() {
        try {
            limitSwitchDA = new DigitalInput(0);
            limitSwitchSA = new DigitalInput(1);
        }  catch (Exception e) {
        	dashboardLog.logError("Error occured while initializing limit switched.");
			dashboardLog.logError(e);
		}
    }

    public static void initProxSensors() {
        try {
            proxSensorLow = new DigitalInput(2);
            proxSensorHigh = new DigitalInput(3);
        }  catch (Exception e) {
        	dashboardLog.logError("Error occured while initializing prox sensors.");
			dashboardLog.logError(e);
		}
    }

    public static void initLimelight() {
        try {
            limelightService = Executors.newSingleThreadScheduledExecutor();
            limelightServo = new Servo(0);
            limelight = new Limelight();
        }  catch (Exception e) {
        	dashboardLog.logError("Error occured while initializing limelight.");
			dashboardLog.logError(e);
		}
    }

    public static void initPressureTransducer() {
        try {
            pressureTransducer = new AnalogInput(3);
        }  catch (Exception e) {
			dashboardLog.logError("Error occured while initializing pressure transducer.");
			dashboardLog.logError(e);
		}
    }
}

