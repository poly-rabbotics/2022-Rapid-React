package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;

public class AHRSGyro {
    public static AHRS gyro;

    public AHRSGyro() {
        gyro = new AHRS();
        
    }

    public double getDegrees() {
        return gyro.getAngle();
    }

    public boolean getMovementStatus() { //returns true if moving or rotating
        return gyro.isMoving() && gyro.isRotating();
    }
    public void reset() {
        gyro.zeroYaw();
        gyro.reset();
    }
 
}