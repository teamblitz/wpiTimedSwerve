package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;

public class SaitekX52Joystick extends GenericHID {

    private final int kX = 0;
    private final int kY = 1;
    private final int kTwist = 5;

    // public enum Button {


    //     public final int value;

    //     Button(int value) {

    //     }
    // }

    public SaitekX52Joystick(int port) {
        super(port);
    }

    public double getX() {
        return getRawAxis(kX);
    }
    
    public double getY() {
        return getRawAxis(kY);
    }

    public double getTwist() {
        return getRawAxis(kTwist);
    }
    
    // public double getButton() {
    //     return getRawButton(button);
    // }
}
