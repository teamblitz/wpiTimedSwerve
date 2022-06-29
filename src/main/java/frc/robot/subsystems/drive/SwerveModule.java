// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;  

//import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Not a subsystem, but a dependancy of a subsystem */
public class SwerveModule {

    // private static final int kEncoderResolution = 4096;
    private double m_offset = 0;
    String m_prefsName;
    String m_dashAbv;

    //JWG talon SRX raw encoder position needs to be converted to radians for Rotation2d
    // private static final double kTicksPerDegree = kEncoderResolution / 360;
    private MotorController m_driveMotor;
    private WPI_TalonSRX m_turningMotor;

    
    // TODO: FIXME: IMPORTANT: Adjust gains for spiderbot
    // DO this in wpi system identification app.
    // See here https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/introduction.html#:~:text=The%20WPILib%20system%20identification%20tool%20consists%20of%20an%20application%20that,data%20back%20to%20the%20application.
    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController m_turningPIDController =
        new PIDController(10,0,0); // Proportional for voltage value, steering twitchyness

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
     *
     * @param driveMotorChannel PWM output for the drive motor.
     * @param turningMotorChannel PWM output for the turning motor.
     * @param prefsName    name for preferences to store encoder offset value
     * @param dashAbv short abbriviation for use in telementry
     */
    // JWG dont have encoder channels, using TalonSRX sensor feedback from analog 5v encoder
    public SwerveModule(
            int driveMotorChannel,
            int turningMotorChannel,
            String prefsName,
            String dashAbv) {
        
        m_prefsName = prefsName;
        m_dashAbv = dashAbv;

        if(driveMotorChannel == Constants.kRearRightDrive)
            m_driveMotor = new WPI_VictorSPX(driveMotorChannel);
        else
            m_driveMotor = new WPI_TalonSRX(driveMotorChannel);

        m_turningMotor = new WPI_TalonSRX(turningMotorChannel);
        m_turningMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
        m_turningMotor.configFeedbackNotContinuous(true, 10);

        loadWheelOffset();
        // could have brake mode set here

        // Limit the PID Controller's input range between -pi and pi and set the input to be continuous.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void loadWheelOffset () {
        m_offset = Preferences.getDouble(m_prefsName, 0);
    }

    public void setWheelOffset () {
        double steerPosition = m_turningMotor.getSelectedSensorPosition(0);
        Preferences.setDouble(m_prefsName, steerPosition);
        m_offset = steerPosition;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */

    private Rotation2d cachedAngle = Rotation2d.fromDegrees(0);
    
    public void setDesiredState(SwerveModuleState desiredState) {
        // The following code snipet caches the rotation of our swerve modules.
        // It does this so we do not instantly snap back to the defalut position after we release the joystick
        // Makes controll a lot easyer
        if (desiredState.speedMetersPerSecond == 0) { // It only does it if we aren't moving
            desiredState.angle = cachedAngle; // Use the cached angle, default 0
        } else {
            cachedAngle = desiredState.angle; // Otherwise cach our current angle
        }
        // Optimize the reference state to avoid spinning further than 90 degrees
        // reading encoder through TalonSRX trying getSelectedSensorPosition(0), where 0 is primary closed loop        
        double degrees = (m_turningMotor.getSelectedSensorPosition(0) - m_offset)/ Constants.kTicksPerDegree;
        double radians = java.lang.Math.toRadians(degrees);
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(radians));
        
        // Calculate the drive output from the drive PID controller. 
        final double driveOutput = state.speedMetersPerSecond;

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = m_turningPIDController.calculate(radians, state.angle.getRadians());

        // JWG without drive encoder, using state.speed with multiplier to convert to voltage
        m_driveMotor.setVoltage(driveOutput * Constants.kDriveSpeedVoltageModifier );
        m_turningMotor.setVoltage(turnOutput);    

    }

    public void displayDashboard() {
        SmartDashboard.putNumber(m_dashAbv + " Encoder", m_turningMotor.getSelectedSensorPosition(0));
        SmartDashboard.putNumber(m_dashAbv + " Wheel Offset", m_offset);
    }
}