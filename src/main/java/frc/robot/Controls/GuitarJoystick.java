package frc.robot.Controls;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotMap;

public class GuitarJoystick {
    private static Joystick joystick = RobotMap.guitarJoystick;

    public static boolean farShot() {
        return joystick.getRawButton(4);
    }

    public static boolean closeShot() {
        return joystick.getRawButton(3);
    }

    public static boolean conveyorIn() {
        return joystick.getRawButton(1);
    }

    public static boolean conveyorOut() {
        return joystick.getRawButton(2);
    }

    public static boolean intake() {
        return joystick.getRawButton(5);
    }
}
