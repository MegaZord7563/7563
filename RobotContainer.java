package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
    // Drive subsystem
    private final DriveSubsystem m_drive = new DriveSubsystem();

    // Driver controller (Xbox)
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public RobotContainer() {
        // Set the default command for the drive subsystem
        // Uses left and right Y axes for tank control
        m_drive.setDefaultCommand(
            new RunCommand(
                () -> m_drive.tankDrive(
                    m_driverController.getLeftY(),
                    m_driverController.getRightY()),
                m_drive));
    }

    /**
     * Returns the autonomous command (can be empty for now).
     */
    public Command getAutonomousCommand() {
        return null; // Replace with a real autonomous command
    }
}