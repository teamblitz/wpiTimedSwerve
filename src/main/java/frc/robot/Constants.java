package frc.robot;

public class Constants {

    // Field Centric Control
    public static boolean kAutoFieldCentric = false;
    public static boolean kTeleFieldCentric = false;

    // steering module encoder
    public static final int kEncoderResolution = 4096;
    public static final double kTicksPerDegree = kEncoderResolution / 360;

    // motors
    public static final int kFrontLeftDrive = 1;
    public static final int kFrontLeftTurning = 2;

    public static final int kFrontRightDrive = 3;
    public static final int kFrontRightTurning = 4;

    public static final int kRearLeftDrive = 8;
    public static final int kRearLeftTurning = 7;

    public static final int kRearRightDrive = 6;
    public static final int kRearRightTurning = 5;

    // Drive speed input voltage modifier
    public static final double kDriveSpeedVoltageModifier = 4.0;

    // Speed Controls
    public static final double kMaxSpeed = 1.5; // 3 meters per second
    public static final double kMaxAngularSpeed = 2 * Math.PI; // Control speed of rotation

    // public static final double kMaxSpeed = 3.0; // 3 meters per second
    // public static final double kMaxAngularSpeed = 4 * Math.PI; // Control speed of rotation


    //Deadband for crappy joystick
    public static final double kDeadband = 0.15;

    //Tornado spin limiter
    public static final double kSpinLimiter = 0.5;

    //Slew rate limiter, Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1
    public static final double kSlewRateLimiter = 3.0;

}
