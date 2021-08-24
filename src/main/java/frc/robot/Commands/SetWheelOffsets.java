package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SwerveDrivetrain;


public class SetWheelOffsets extends CommandBase {
        // super(subsystem::SetAllWheelOffsets);
        
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
            System.out.println("SetWheelOffsets Command");
        }

        

        @Override
        public boolean isFinished() {
            return true;
        }
}
