// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package edu.wpi.first.wpilibj.examples.swervebot;
package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.SetWheelOffsets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final SwerveDrivetrain m_swerve = new SwerveDrivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Override
  public void robotInit() {
    

    // TODO Auto-generated method stub
    super.robotInit();
    LiveWindow.disableAllTelemetry();
    SmartDashboard.putData("SetWheelOffsets", new SetWheelOffsets(m_swerve));
  }

  @Override
  public void robotPeriodic() {
    if (RobotController.getUserButton() == true)
      m_swerve.SetAllWheelOffsets();
    m_swerve.ShowDashboardData();

    CommandScheduler.getInstance().run();
  }


  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    // m_swerve.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(false);
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double yLeft;
    double xLeft;
    double xRight;

    SmartDashboard.putNumber("Left Y", m_controller.getY(GenericHID.Hand.kLeft));
    SmartDashboard.putNumber("Left X", m_controller.getX(GenericHID.Hand.kLeft));
    SmartDashboard.putNumber("Right X", m_controller.getX(GenericHID.Hand.kRight));

    if (Math.abs(m_controller.getY(GenericHID.Hand.kLeft)) < .15) 
      yLeft = 0.0;
    else 
      yLeft = m_controller.getY(GenericHID.Hand.kLeft);
    
    if (Math.abs(m_controller.getX(GenericHID.Hand.kLeft)) < .15) 
      xLeft = 0.0;
    else 
      xLeft = m_controller.getX(GenericHID.Hand.kLeft);

    if (Math.abs(m_controller.getX(GenericHID.Hand.kRight)) < .15) 
      xRight = 0.0;
    else 
      xRight = m_controller.getX(GenericHID.Hand.kRight) * .2;  // slow the tornado spin

    final var xSpeed = -m_xspeedLimiter.calculate(yLeft) * SwerveDrivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -m_yspeedLimiter.calculate(xLeft) * SwerveDrivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -m_rotLimiter.calculate(xRight) * SwerveDrivetrain.kMaxAngularSpeed;

    // m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
    m_swerve.drive(xSpeed, ySpeed, rot);
  }
}