package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pigeon {
    static final int PIGEON_CAN_ID = 0;

    /**
     * The pigeons single instance.
     */
    static Pigeon instance = new Pigeon();

    private Pigeon2 pigeonController;

    private Pigeon() {
        pigeonController = new Pigeon2(PIGEON_CAN_ID);

        // Zero all positions on initialization.
        pigeonController.configMountPose(0, 0, 0);
    }


    /**
     * Put all degree values from the pigeon to smart dashboard.
     */
    public static void putDegrees() {
        SmartDashboard.putNumber("pigeon yaw", instance.pigeonController.getYaw());
        SmartDashboard.putNumber("pigeon pitch", instance.pigeonController.getPitch());
        SmartDashboard.putNumber("pigeon roll", instance.pigeonController.getRoll());
    }
}
