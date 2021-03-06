// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot; 

import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import com.kauailabs.navx.frc.AHRS;

// Represents a swerve drive style drivetrain
public class SwerveDrivetrain {
  // public static final double kMaxSpeed = 3.0; // 3 meters per second
  // public static final double kMaxAngularSpeed = 4 * Math.PI; // Control speed of rotation

  private final Translation2d m_frontLeftLocation = new Translation2d(0.3175, 0.27305);
  private final Translation2d m_frontRightLocation = new Translation2d(0.3175, -0.27305);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.3175, 0.27305);
  private final Translation2d m_backRightLocation = new Translation2d(-0.3175, -0.27305);

  private final SwerveModule m_frontLeft = new SwerveModule(Constants.kFrontLeftDrive, Constants.kFrontLeftTurning, "FrontLeft");  // drive, turning, prefs name
  private final SwerveModule m_frontRight = new SwerveModule(Constants.kFrontRightDrive, Constants.kFrontRightTurning, "FrontRight");
  private final SwerveModule m_backLeft = new SwerveModule(Constants.kRearLeftDrive, Constants.kRearLeftTurning, "BackLeft");
  private final SwerveModule m_backRight = new SwerveModule(Constants.kRearRightDrive, Constants.kRearRightTurning, "BackRight");

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  public SwerveDrivetrain() {
      // Reset gyro when drivetrain is initialized
      m_gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
               ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void SetAllWheelOffsets () {
    m_frontLeft.SetWheelOffset();
    m_frontRight.SetWheelOffset();
    m_backLeft.SetWheelOffset();
    m_backRight.SetWheelOffset();
    // System.out.println("SetWheelOffsets Drivetrain");
  }

  public void ShowDashboardData () {
    m_frontLeft.displayDashboard(m_gyro);
    m_frontRight.displayDashboard(m_gyro);
    m_backLeft.displayDashboard(m_gyro);
    m_backRight.displayDashboard(m_gyro);
  }

  public void ResetGyro () {
    m_gyro.reset();
  }

}