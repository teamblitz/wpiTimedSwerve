// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;  

//import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
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

  //JWG talon SRX raw encoder position needs to be converted to radians for Rotation2d
  // private static final double kTicksPerDegree = kEncoderResolution / 360;
  private MotorController m_driveMotor;
  private WPI_TalonSRX m_turningMotor;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_turningPIDController =
      new PIDController(10,0,0); // Proportional for voltage value, steering twitchyness

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param prefsName  name for preferences to store encoder offset value
   */
  // JWG dont have encoder channels, using TalonSRX sensor feedback from analog 5v encoder
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      String prefsName) {
    
    m_prefsName = prefsName;

    if(driveMotorChannel == Constants.kRearRightDrive)
      m_driveMotor = new WPI_VictorSPX(driveMotorChannel);
    else
      m_driveMotor = new WPI_TalonSRX(driveMotorChannel);

    m_turningMotor = new WPI_TalonSRX(turningMotorChannel);
    m_turningMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    m_turningMotor.configFeedbackNotContinuous(true, 10);

    LoadWheelOffset();
    // could have brake mode set here

    // Limit the PID Controller's input range between -pi and pi and set the input to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void LoadWheelOffset () {
    m_offset = Preferences.getDouble(m_prefsName, 0);
  }

  public void SetWheelOffset () {
    double steerPosition = m_turningMotor.getSelectedSensorPosition(0);
   Preferences.setDouble(m_prefsName, steerPosition);
    m_offset = steerPosition;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */

  private Rotation2d cashedAngle = Rotation2d.fromDegrees(0);
  
  public void setDesiredState(SwerveModuleState desiredState) {
    if (desiredState.speedMetersPerSecond == 0) {
      desiredState.angle = cashedAngle;
    } else {
      cashedAngle = desiredState.angle;
    }
    //desiredState.angle = desiredState.speedMetersPerSecond == 0 ? cashedAngle : desiredState.angle
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

  public void displayDashboard(AHRS gyro) {

  // OUTPUT
    SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());

    if (m_turningMotor.getDeviceID() == Constants.kFrontLeftTurning) {
         // Front Left
         SmartDashboard.putNumber("FL Encoder", m_turningMotor.getSelectedSensorPosition(0));
         SmartDashboard.putNumber("FL Wheel Offset", m_offset);
     }
    else if (m_turningMotor.getDeviceID() == Constants.kFrontRightTurning) {
        // Front Right
        SmartDashboard.putNumber("FR Encoder", m_turningMotor.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("FR Wheel Offset", m_offset);
    
    }
    else if (m_turningMotor.getDeviceID() == Constants.kRearLeftTurning) {
        // Rear Left
        SmartDashboard.putNumber("RL Encoder", m_turningMotor.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("RL Wheel Offset", m_offset);
    }
    else if (m_turningMotor.getDeviceID() == Constants.kRearRightTurning) {
         // Rear Right
         SmartDashboard.putNumber("RR Encoder", m_turningMotor.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("RR Wheel Offset", m_offset);
     }
    else
        System.out.println("Smartdashboard miss on steering motorcontroller");


  }

}