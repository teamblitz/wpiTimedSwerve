// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive; 

//import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.kauailabs.navx.frc.AHRS;

// Represents a swerve drive style drivetrain
public class SwerveDrivetrain extends SubsystemBase {
  // public static final double kMaxSpeed = 3.0; // 3 meters per second
  // public static final double kMaxAngularSpeed = 4 * Math.PI; // Control speed of rotation

  private final Translation2d m_frontLeftLocation = new Translation2d(0.3175, 0.27305);
  private final Translation2d m_frontRightLocation = new Translation2d(0.3175, -0.27305);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.3175, 0.27305);
  private final Translation2d m_backRightLocation = new Translation2d(-0.3175, -0.27305);

  private final SwerveModule m_frontLeft = new SwerveModule(Constants.kFrontLeftDrive, Constants.kFrontLeftTurning, "FrontLeft", "FL");  // drive, turning, prefs name
  private final SwerveModule m_frontRight = new SwerveModule(Constants.kFrontRightDrive, Constants.kFrontRightTurning, "FrontRight", "FR");
  private final SwerveModule m_backLeft = new SwerveModule(Constants.kRearLeftDrive, Constants.kRearLeftTurning, "BackLeft", "BL");
  private final SwerveModule m_backRight = new SwerveModule(Constants.kRearRightDrive, Constants.kRearRightTurning, "BackRight", "BR");

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
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
               ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setAllWheelOffsets () {
    m_frontLeft.setWheelOffset();
    m_frontRight.setWheelOffset();
    m_backLeft.setWheelOffset();
    m_backRight.setWheelOffset();
    // System.out.println("SetWheelOffsets Drivetrain");
  }
  @Override
  public void periodic () {

    SmartDashboard.putNumber("Gyro Yaw", m_gyro.getYaw());
    SmartDashboard.putNumber("Gyro Angle", m_gyro.getAngle());
    SmartDashboard.putNumber("Gyro AngleAdjustment", m_gyro.getAngleAdjustment());
    SmartDashboard.putNumber("Gyro Compass", m_gyro.getCompassHeading());

    m_frontLeft.displayDashboard();
    m_frontRight.displayDashboard();
    m_backLeft.displayDashboard();
    m_backRight.displayDashboard();
  }

  public void resetGyro () {
    m_gyro.reset();
  }
}