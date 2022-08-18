package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;

public class SaitekX52Joystick extends GenericHID {

    private final int kX = 0;
    private final int kY = 1;
    private final int kTwist = 5;


    public enum Axis {
        kXAxis(0),
        kYAxis(1),
        /** Throtle */
        kZAxis(2),
        /** Small Dial */
        kXRot(3),
        /** Large Dial */
        kYRot(4),
        /** Twist */
        kZRot(5),
        kSlider(6);




        public final int value;

        Axis(int value) {
            this.value=value;
        }
    }

    public enum Button {
        kT1(9),
        kT2(10),
        kT3(11),
        kT4(12),
        kT5(13),
        kT6(24);


        public final int value;

        Button(int value) {
            this.value=value;
        }
    }

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
