/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.SwerveDrivetrain;


/**
* This class is where the bulk of the robot should be declared.  Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls).  Instead, the structure of the robot
* (including subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {

    private final XboxController m_controller = new XboxController(0);
    private final SwerveDrivetrain m_drive = new SwerveDrivetrain();

    public RobotContainer() {
        configureSubsystems();
        configureButtonBindings();
        setDefaultCommands();
    }

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(Constants.kSlewRateLimiter);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(Constants.kSlewRateLimiter);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.kSlewRateLimiter);

    private void setDefaultCommands() {
        // Set defalut command for drive
        m_drive.setDefaultCommand(new RunCommand(() -> {

        m_drive.drive(-m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), Constants.kDeadband)) * Constants.kMaxSpeed,
                      -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), Constants.kDeadband)) * Constants.kMaxSpeed, 
                      -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), Constants.kDeadband)) * Constants.kMaxAngularSpeed, 
                      false);
        }, m_drive));
        
    }

    private void driveWithJoystick(boolean fieldRelative) {
        
      }


    private void configureSubsystems() {
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
    * instantiating a {@link GenericHID} or one of its subclasses ({@link
    * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
    * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
    */
    private void configureButtonBindings() {

    }

    public Command getAutonomousCommands() { // Autonomous code goes here
        return null;
    }
}

