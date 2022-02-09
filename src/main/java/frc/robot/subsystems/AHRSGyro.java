package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;

public class AHRSGyro {
    public static AHRS gyro;

    public AHRSGyro() {
        gyro = new AHRS();
        gyro.reset();
        gyro.calibrate();
    }

    public double getDegrees() {
        return gyro.getAngle();
    }

    public void reset() {
        gyro.reset();
    }

}