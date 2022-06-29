package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.SwerveDrivetrain;

// Command for button on dashboard to reset the wheel offsets
public class SetWheelOffsets extends CommandBase {        
        
        private final SwerveDrivetrain m_swerve;

        public SetWheelOffsets(SwerveDrivetrain subsystem) {
            m_swerve = subsystem;            
            // addRequirements(m_swerve);
        }

        @Override
        public boolean runsWhenDisabled() {
            return true;
        }

        @Override
        public void initialize() {
            m_swerve.SetAllWheelOffsets();
            // System.out.println("SetWheelOffsets Command");
        }        

        @Override
        public boolean isFinished() {
            return true;
        }
}
