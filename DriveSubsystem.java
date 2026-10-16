package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    // Left side motors
    private final PWMSparkMax m_leftLeader = new PWMSparkMax(DriveConstants.kLeftMotor1Port);
    private final PWMSparkMax m_leftFollower = new PWMSparkMax(DriveConstants.kLeftMotor2Port);

    // Right side motors
    private final PWMSparkMax m_rightLeader = new PWMSparkMax(DriveConstants.kRightMotor1Port);
    private final PWMSparkMax m_rightFollower = new PWMSparkMax(DriveConstants.kRightMotor2Port);

    // DifferentialDrive object – receives references to the leader motors
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);

    // Encoders
    private final Encoder m_leftEncoder = new Encoder(
        DriveConstants.kLeftEncoderPorts[0],
        DriveConstants.kLeftEncoderPorts[1],
        DriveConstants.kLeftEncoderReversed);
    private final Encoder m_rightEncoder = new Encoder(
        DriveConstants.kRightEncoderPorts[0],
        DriveConstants.kRightEncoderPorts[1],
        DriveConstants.kRightEncoderReversed);

    public DriveSubsystem() {
        // Configure left follower to follow the leader
        m_leftLeader.addFollower(m_leftFollower);

        // Configure right follower to follow the leader
        m_rightLeader.addFollower(m_rightFollower);

        // Invert right side motors so positive values drive the robot forward
        m_rightLeader.setInverted(true);
        // The follower inherits the leader's inversion, so no need to invert it again

        // Set encoder distance per pulse
        m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

        // Reset encoders
        resetEncoders();
    }

    /**
     * Main tank drive control method.
     * @param leftSpeed  Left side speed (-1.0 to 1.0)
     * @param rightSpeed Right side speed (-1.0 to 1.0)
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        m_drive.tankDrive(leftSpeed, rightSpeed);
    }

    /** Resets the encoder counts. */
    public void resetEncoders() {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }

    /** Returns the distance traveled by the left side (meters). */
    public double getLeftDistance() {
        return m_leftEncoder.getDistance();
    }

    /** Returns the distance traveled by the right side (meters). */
    public double getRightDistance() {
        return m_rightEncoder.getDistance();
    }

    /** Returns the left side rate (m/s). */
    public double getLeftRate() {
        return m_leftEncoder.getRate();
    }

    /** Returns the right side rate (m/s). */
    public double getRightRate() {
        return m_rightEncoder.getRate();
    }

    @Override
    public void periodic() {
        // Update odometry here if needed
        // m_odometry.update(m_gyro.getRotation2d(), getLeftDistance(), getRightDistance());
    }
}