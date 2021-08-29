// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;  

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import edu.wpi.first.wpilibj.;
public class SwerveModule {

  private static final int kEncoderResolution = 4096;
  private double m_offset = 0;
  String m_prefsName;

  //JWG talon SRX raw encoder position needs to be converted to radians for Rotation2d
  private static final double kTicksPerDegree = kEncoderResolution / 360;
  private SpeedController m_driveMotor;
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

    if(driveMotorChannel==6)
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
    Preferences prefs = Preferences.getInstance();
    m_offset = prefs.getDouble(m_prefsName, 0);
  }

  public void SetWheelOffset () {
    Preferences prefs = Preferences.getInstance();
    double steerPosition = m_turningMotor.getSelectedSensorPosition(0);
    prefs.putDouble(m_prefsName, steerPosition);
    m_offset = steerPosition;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    // reading encoder through TalonSRX trying getSelectedSensorPosition(0), where 0 is primary closed loop        
    double degrees = (m_turningMotor.getSelectedSensorPosition(0) - m_offset)/ kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(radians));
    
    // Calculate the drive output from the drive PID controller. 
    final double driveOutput = state.speedMetersPerSecond;

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(radians, state.angle.getRadians());

    // JWG without drive encoder, just use state.speed converted to voltage
    m_driveMotor.setVoltage(driveOutput * 4.0 );
    m_turningMotor.setVoltage(turnOutput);    

  }

  public void displayDashboard(AHRS gyro) {

  // OUTPUT
    SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());

    if (m_turningMotor.getDeviceID() == 2) {
         // Front Left
         SmartDashboard.putNumber("FL Encoder", m_turningMotor.getSelectedSensorPosition(0));
         SmartDashboard.putNumber("FL Wheel Offset", m_offset);
     }
    else if (m_turningMotor.getDeviceID() == 4) {
        // Front Right
        SmartDashboard.putNumber("FR Encoder", m_turningMotor.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("FR Wheel Offset", m_offset);
    
    }
    else if (m_turningMotor.getDeviceID() == 7) {
        // Rear Left
        SmartDashboard.putNumber("RL Encoder", m_turningMotor.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("RL Wheel Offset", m_offset);
    }
    else if (m_turningMotor.getDeviceID() == 5) {
         // Rear Right
         SmartDashboard.putNumber("RR Encoder", m_turningMotor.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("RR Wheel Offset", m_offset);
     }
    else
        System.out.println("Smartdashboard miss on steering motorcontroller");


  }

}