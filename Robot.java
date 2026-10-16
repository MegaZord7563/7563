package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        // Instantiate RobotContainer – this is where subsystems and commands are configured
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        // Run the command scheduler
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        // Cancel the autonomous command when teleop starts, if necessary
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }
}