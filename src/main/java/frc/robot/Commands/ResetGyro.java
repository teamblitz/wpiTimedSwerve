package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.SwerveDrivetrain;

// Command for button on dashboard to reset the gyro yaw
public class ResetGyro extends CommandBase {        
                
        private final SwerveDrivetrain m_swerve;

        public ResetGyro(SwerveDrivetrain subsystem) {
            m_swerve = subsystem;            
            // addRequirements(m_swerve);
        }

        @Override
        public boolean runsWhenDisabled() {
            return true;
        }

        @Override
        public void initialize() {
            m_swerve.resetGyro();        
        }        

        @Override
        public boolean isFinished() {
            return true;
        }
}
