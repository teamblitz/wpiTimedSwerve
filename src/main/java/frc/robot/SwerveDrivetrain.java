// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot; 

// import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.AnalogGyro;
// import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
// import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;

/** Represents a swerve drive style drivetrain. */
public class SwerveDrivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.3175, 0.27305);
  private final Translation2d m_frontRightLocation = new Translation2d(0.3175, -0.27305);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.3175, 0.27305);
  private final Translation2d m_backRightLocation = new Translation2d(-0.3175, -0.27305);

  // private final SwerveModule m_frontLeft = new SwerveModule(1, 2, 0, 1, 2, 3);
  // private final SwerveModule m_frontRight = new SwerveModule(3, 4, 4, 5, 6, 7);
  // private final SwerveModule m_backLeft = new SwerveModule(5, 6, 8, 9, 10, 11);
  // private final SwerveModule m_backRight = new SwerveModule(7, 8, 12, 13, 14, 15);
  private final SwerveModule m_frontLeft = new SwerveModule(1, 2);  // drive, turning
  private final SwerveModule m_frontRight = new SwerveModule(3, 4);
  private final SwerveModule m_backLeft = new SwerveModule(5, 6);
  private final SwerveModule m_backRight = new SwerveModule(7, 8);

  // private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

// JWG not dealing with feild centric for now, dont need gyro           
//  private final SwerveDriveOdometry m_odometry =
//      // new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());
//      new SwerveDriveOdometry(m_kinematics, getGyro());

  // public SwerveDrivetrain() {
  //     m_gyro.reset();
  // }


  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  // public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    public void drive(double xSpeed, double ySpeed, double rot) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            new ChassisSpeeds(xSpeed, ySpeed, rot));
  //          fieldRelative
  //             ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyro())
  //              : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  // JWG not dealing with feild centric for now
  /** Updates the field relative position of the robot. */
 // public void updateOdometry() {
 //   m_odometry.update(
 //       getGyro(),
 //       m_frontLeft.getState(),
 //       m_frontRight.getState(),
 //       m_backLeft.getState(),
 //      m_backRight.getState());
 // }
}