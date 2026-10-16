// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OdometryConstants;
import frc.robot.Constants.SimConstants;

public class DriveSubsystem extends SubsystemBase implements AutoCloseable
{
  private final SparkMax leftLeader;
  //SparkMax leftFollower;
  private final SparkMax rightLeader;
  //SparkMax rightFollower;
  private final DifferentialDrive differentialDrive;
  // Constants for PID control



  private double kP1 = 0.0180;//0.00795

  PIDController pidController = new PIDController(kP1, 0.0, 0.00); // Initialize PID controller with kP, kI, kD


  private final AHRS gyroAhrs;// = new AHRS(NavXComType.kMXP_SPI,AHRS.NavXUpdateRate.k50Hz); // Adjusted to use a valid constructor

  // --- Odometry -----------------------------------------------------------------------
  // The brushed SPARK MAXes expose no built-in encoder, so wheel travel comes from
  // quadrature encoders wired to the roboRIO DIO ports. Odometry fuses those two distances
  // with the gyro heading into a field-relative Pose2d: the encoders say how far each side
  // rolled, the gyro says which way the robot was pointing while it rolled.
  private final Encoder leftEncoder =
      new Encoder(OdometryConstants.kLeftEncoderChannelA,
                  OdometryConstants.kLeftEncoderChannelB,
                  OdometryConstants.kLeftEncoderReversed,
                  EncodingType.k4X);
  private final Encoder rightEncoder =
      new Encoder(OdometryConstants.kRightEncoderChannelA,
                  OdometryConstants.kRightEncoderChannelB,
                  OdometryConstants.kRightEncoderReversed,
                  EncodingType.k4X);

  /** Not used by odometry itself, but it is the standard way to convert pose speeds to wheels. */
  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(OdometryConstants.kTrackWidthMeters);

  private final DifferentialDriveOdometry odometry;

  private final Field2d field = new Field2d();

  // --- Simulation ---------------------------------------------------------------------
  // The NavX has no simulation model (the Studica AHRS registers no SimDevice) and the
  // simulated encoders have nothing to count, so there is no real sensor to simulate from.
  // DifferentialDrivetrainSim models the drivetrain from motor voltages instead, and its
  // output is written back into the heading and the EncoderSims, so odometry runs off
  // exactly the same inputs in simulation as it does on the robot.
  private final DifferentialDrivetrainSim driveSim =
      DifferentialDrivetrainSim.createKitbotSim(SimConstants.kMotor,
                                                SimConstants.kGearing,
                                                SimConstants.kWheelSize,
                                                null); // null = no measurement noise
  private final EncoderSim leftEncoderSim;
  private final EncoderSim rightEncoderSim;
  /** Subtracted from the simulated heading so resetGyro() zeroes it like the real gyro. */
  private double simYawOffsetDeg = 0.0;


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem()
  {
    leftLeader = new SparkMax(DriveConstants.kLeftMotorPort, MotorType.kBrushed);
    //leftFollower = new SparkMax(1, MotorType.kBrushed);
    rightLeader = new SparkMax(DriveConstants.kRightMotorPort, MotorType.kBrushed);
    //rightFollower = new SparkMax(2, MotorType.kBrushed);
    gyroAhrs = new AHRS(NavXComType.kMXP_SPI,AHRS.NavXUpdateRate.k50Hz);

    // Initialize the DifferentialDrive with the left and right leaders
    differentialDrive = new DifferentialDrive(leftLeader, rightLeader);

    // Scales raw counts into metres, so getDistance() returns metres of wheel travel. This
    // must be set before odometry is seeded, or the first pose would come from raw counts.
    leftEncoder.setDistancePerPulse(OdometryConstants.kEncoderDistancePerPulse);
    rightEncoder.setDistancePerPulse(OdometryConstants.kEncoderDistancePerPulse);

    // Position tolerance in degrees, rate tolerance in degrees per second. Without the
    // rate tolerance the controller reports "at setpoint" while still rotating fast,
    // which ends the turn command early and lets the robot coast past the heading.
    pidController.setTolerance(DriveConstants.kHeadingToleranceDeg,
                               DriveConstants.kHeadingRateToleranceDegPerSec);

    // Headings wrap at +/-180, so let the controller turn the short way around.
    pidController.enableContinuousInput(-180.0, 180.0);

    // setDefaultNumber (not putNumber) so a value typed on the dashboard survives a
    // code restart instead of being reset to the default on every boot.
    SmartDashboard.setDefaultNumber("Heading SetPoint", 0.0);
    SmartDashboard.setDefaultNumber("KP Heading", kP1);


    /*
     * Create new SPARK MAX configuration objects. These will store the
     * configuration parameters for the SPARK MAXes that we will set below.
     */
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();

    //SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    //SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

    /*
     * Set parameters that will apply to all SPARKs. We will also use this as
     * the left leader config.
     */
    globalConfig
        .smartCurrentLimit(DriveConstants.currentLimit) // Set the current limit for the motors
        .idleMode(DriveConstants.kIdleModeLeftMotor); // Set the idle mode for the motors;

    // Apply the global config and invert since it is on the opposite side
    rightLeaderConfig
        .apply(globalConfig)
        .idleMode(DriveConstants.kIdleModeRightMotor) // Override the left leader's idle mode
        .inverted(true);

    // Apply the global config and set the leader SPARK for follower mode
    /*leftFollowerConfig
        .apply(globalConfig)
        .follow(leftLeader);

    // Apply the global config and set the leader SPARK for follower mode
    rightFollowerConfig
        .apply(globalConfig)
        .follow(rightLeader);*/

    /*
     * Apply the configuration to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    leftLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    gyroAhrs.reset();// Reset the gyro to ensure it starts at 0 degrees
    gyroAhrs.zeroYaw(); // Reset the yaw to 0 degrees

    leftEncoder.reset();
    rightEncoder.reset();

    if (RobotBase.isSimulation())
    {
      leftEncoderSim = new EncoderSim(leftEncoder);
      rightEncoderSim = new EncoderSim(rightEncoder);

      // Place the simulated robot where odometry is about to be told the robot is, so the
      // estimate and the simulated truth start out agreeing.
      driveSim.setPose(FieldConstants.kStartingPose);
    }
    else
    {
      leftEncoderSim = null;
      rightEncoderSim = null;
    }

    // Seeded last, so it starts from already-zeroed encoders and the simulated start pose.
    odometry = new DifferentialDriveOdometry(getRotation2d(),
                                             getLeftDistanceMeters(),
                                             getRightDistanceMeters(),
                                             FieldConstants.kStartingPose);

    // Published in simulation and on the robot alike -- with odometry there is now a real
    // pose to show in either case. Select "2026 Rebuilt" as the builtin image in the Sim
    // GUI's Field2d window.
    SmartDashboard.putData("Field", field);

  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run

    // Live-tune the heading gain. Assigning kP1 alone would have no effect, the gain has
    // to be pushed into the controller with setP. Non-positive values are ignored so an
    // empty or zeroed dashboard entry cannot silently disable the turn.
    double dashboardKp = SmartDashboard.getNumber("KP Heading", kP1);
    if (dashboardKp > 0.0 && dashboardKp != kP1)
    {
      kP1 = dashboardKp;
      pidController.setP(kP1);
    }

    // Integrate wheel travel along the current heading. This has to run every scheduler
    // loop: odometry is dead reckoning, so a skipped update is distance the pose can
    // never recover.
    odometry.update(getRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters());

    Pose2d pose = getPose();
    field.setRobotPose(pose);

    SmartDashboard.putNumber("Current Heading", this.getHeading());
    SmartDashboard.putNumber("Gyro Yaw", this.getYaw()); // Update the yaw value to keep it fresh

    SmartDashboard.putNumber("Odometry X (m)", pose.getX());
    SmartDashboard.putNumber("Odometry Y (m)", pose.getY());
    SmartDashboard.putNumber("Odometry Heading (deg)", pose.getRotation().getDegrees());
    SmartDashboard.putNumber("Left Distance (m)", getLeftDistanceMeters());
    SmartDashboard.putNumber("Right Distance (m)", getRightDistanceMeters());

  }

  /**
   * Advances the simulated drivetrain and feeds its output back into the simulated sensors.
   * Called by the CommandScheduler only in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
    // DifferentialDrive drives both sides positive for forward and relies on the right
    // SPARK's own inversion, so get() already reports "positive = forward" for both.
    double batteryVolts = RobotController.getBatteryVoltage();
    driveSim.setInputs(leftLeader.get() * batteryVolts, rightLeader.get() * batteryVolts);

    driveSim.update(0.020); // 20 ms, matching the default TimedRobot period

    // Write the model's wheel travel into the encoders odometry reads. Distances are set
    // directly rather than counts, so the reversal flags do not apply here -- both sides
    // read positive-forward, exactly as a correctly wired encoder does on the robot.
    //
    // Note what this does NOT test: setDistance() converts metres to counts with the same
    // distance-per-pulse that getDistance() converts back with, so the scaling cancels out.
    // Simulation therefore cannot validate kWheelDiameterMeters or the encoder CPR -- those
    // have to be measured on the real robot and checked by driving a known distance.
    leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
    leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
    rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());

    // The model's own pose, drawn alongside the estimate. The gap between the two IS the
    // odometry error, which is the only honest way to see dead-reckoning drift.
    field.getObject("Sim Truth").setPose(driveSim.getPose());
  }


  /**
   * Drives the motors based on the forward and rotation values.
   *
   * @param forward The forward speed, where positive is forward and negative is backward.
   * @param rotation The rotation rate, where positive is counterclockwise and negative is clockwise.
   */
  public void driveMotor(double forward, double rotation)
  {
    differentialDrive.arcadeDrive(forward, rotation);
  }

  public boolean pidAtSetPoint()
  {
    // This method checks if the PID controller is within the tolerance
    // It can be used to determine if the robot has reached the target heading
    return pidController.atSetpoint();
  }

  /**
   * Stops the motors.
   */
  public void stopMotors()
  {
    // Stop the robot by setting both forward and rotation to 0
    differentialDrive.tankDrive(0, 0);
  }

  /**
   * Drives the robot to a specific heading.
   * @param heading The target heading in degrees.
   */
  public void drive2Heading(double heading)
  {
    // Turns in place with a P controller on the gyro yaw. Yaw (not getAngle) is used
    // because the controller has continuous input enabled over [-180, 180].
    double currentHeading = this.getYaw();

    pidController.setSetpoint(heading); // Set the target heading for the PID controller
    double output = pidController.calculate(currentHeading); // Calculate the output based on the current heading

    // Clamp explicitly rather than relying on DifferentialDrive's internal clamp, so the
    // turn authority is a named, tunable value. Lower it to reduce overshoot.
    output = MathUtil.clamp(output, -DriveConstants.kMaxTurnOutput, DriveConstants.kMaxTurnOutput);

    differentialDrive.tankDrive(-output, output); // Drive the robot towards the target heading


  }

  // --- Odometry -----------------------------------------------------------------------

  /**
   * The robot's field-relative position and heading, as estimated by odometry.
   * @return The current pose, in metres.
   */
  public Pose2d getPose()
  {
    return odometry.getPoseMeters();
  }

  /** @return Distance travelled by the left wheels since the last reset, in metres. */
  public double getLeftDistanceMeters()
  {
    return leftEncoder.getDistance();
  }

  /** @return Distance travelled by the right wheels since the last reset, in metres. */
  public double getRightDistanceMeters()
  {
    return rightEncoder.getDistance();
  }

  /** @return The average of the two sides, which is how far the robot itself has driven. */
  public double getAverageDistanceMeters()
  {
    return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
  }

  /** @return Current wheel speeds, in metres per second. */
  public DifferentialDriveWheelSpeeds getWheelSpeeds()
  {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  /** @return The kinematics model built from the measured track width. */
  public DifferentialDriveKinematics getKinematics()
  {
    return kinematics;
  }

  /**
   * The heading in the convention odometry and every other WPILib pose API expect:
   * counter-clockwise positive. The NavX reports clockwise positive, hence the negation --
   * getting this sign wrong mirrors the whole field, so it lives in exactly one place.
   *
   * <p>This is deliberately NOT the same as {@link #getHeading()} and {@link #getYaw()}, which
   * keep the NavX's clockwise-positive convention because drive2Heading's setpoints are written
   * in it. On the real robot the two are therefore negatives of each other, and the dashboard
   * shows "Current Heading" and "Odometry Heading (deg)" with opposite signs. That is expected.
   *
   * @return The current heading as a Rotation2d.
   */
  public Rotation2d getRotation2d()
  {
    if (RobotBase.isSimulation())
    {
      // DifferentialDrivetrainSim already reports counter-clockwise positive.
      return Rotation2d.fromDegrees(driveSim.getHeading().getDegrees() - simYawOffsetDeg);
    }
    return Rotation2d.fromDegrees(-gyroAhrs.getAngle());
  }

  /**
   * Declares where the robot actually is, for example at the start of autonomous. Odometry
   * cannot work this out on its own -- it only ever accumulates changes.
   *
   * @param pose The robot's true field-relative pose.
   */
  public void resetOdometry(Pose2d pose)
  {
    leftEncoder.reset();
    rightEncoder.reset();

    if (RobotBase.isSimulation())
    {
      // setPose also zeroes the model's wheel positions, so push that through to the
      // simulated encoders now. Leaving them stale would hand the next update() the whole
      // pre-reset distance as if the robot had just driven it.
      driveSim.setPose(pose);
      leftEncoderSim.setDistance(0.0);
      rightEncoderSim.setDistance(0.0);
      leftEncoderSim.setRate(0.0);
      rightEncoderSim.setRate(0.0);
    }

    // resetPosition stores the difference between the gyro reading and the pose heading,
    // so the gyro does NOT have to be zeroed for this to be correct.
    odometry.resetPosition(getRotation2d(),
                           getLeftDistanceMeters(),
                           getRightDistanceMeters(),
                           pose);
  }

  /** Resets odometry to the configured starting pose. */
  public void resetOdometry()
  {
    resetOdometry(FieldConstants.kStartingPose);
  }

  /** Resets the drive encoders to zero without moving the pose estimate. */
  public void resetEncoders()
  {
    leftEncoder.reset();
    rightEncoder.reset();

    if (RobotBase.isSimulation())
    {
      leftEncoderSim.setDistance(0.0);
      rightEncoderSim.setDistance(0.0);
    }

    // Odometry tracks the CHANGE in these distances, so it has to be told they restarted
    // from zero, or the next update would read the reset as a large jump backwards.
    odometry.resetPosition(getRotation2d(),
                           getLeftDistanceMeters(),
                           getRightDistanceMeters(),
                           getPose());
  }

  /**
   * Gets the current heading of the robot.
   * @return The current heading in degrees.
   */
  public double getHeading()
  {
    if (RobotBase.isSimulation())
    {
      // Continuous (unwrapped) simulated heading, matching getAngle()'s behaviour.
      return driveSim.getHeading().getDegrees() - simYawOffsetDeg;
    }
    // Returns the current heading of the robot
    return gyroAhrs.getAngle();
  }
  /**
   * Gets the current yaw of the robot.
   * @return The current yaw in degrees, wrapped to [-180, 180].
   */
  public double getYaw()
  {
    if (RobotBase.isSimulation())
    {
      // Wrapped like the real AHRS, which is what drive2Heading's continuous input expects.
      return MathUtil.inputModulus(driveSim.getHeading().getDegrees() - simYawOffsetDeg,
                                   -180.0, 180.0);
    }
    // Returns the current yaw of the robot
    return gyroAhrs.getYaw();
  }

  /**
   * Gets the current rate of the gyro.
   * @return The current rate in degrees per second.
   */
  public double getRate()
  {
    // Returns the current rate of the gyro
    return gyroAhrs.getRate();
  }

  /**
   * Resets the gyro to zero.
   * This method is used to reset the gyro's angle and yaw to zero.
   */
  public void resetGyro()
  {
    // Resets the gyro to zero
    gyroAhrs.reset();
    gyroAhrs.zeroYaw(); // Reset the yaw to 0 degrees

    // Zero the simulated heading too, without teleporting the robot on the field.
    simYawOffsetDeg = driveSim.getHeading().getDegrees();

    pidController.reset(); // Clear stale error so atSetpoint() cannot report true immediately

    // Re-seed odometry at the pose it already holds. Odometry stores a fixed gyro offset
    // when it is seeded, so without this the heading reference would jump by however far
    // the robot had turned and drag the pose around with it. The pose is unchanged here;
    // only drive2Heading's reference moves, which is the whole point of zeroing the gyro.
    odometry.resetPosition(getRotation2d(),
                           getLeftDistanceMeters(),
                           getRightDistanceMeters(),
                           getPose());
  }

  /**
   * The pose of the simulated drivetrain model, which is the closest thing to ground truth
   * available off-robot. Comparing it with {@link #getPose()} is how odometry drift is
   * measured; on a real robot the model is never updated, so this is meaningless there.
   *
   * @return The simulated robot pose.
   */
  public Pose2d getSimulatedPose()
  {
    return driveSim.getPose();
  }

  /**
   * Releases the HAL resources this subsystem allocated. The robot program never calls this --
   * it exists so unit tests can construct a DriveSubsystem repeatedly without the DIO channels,
   * CAN IDs and SPI port staying claimed by the previous test.
   */
  @Override
  public void close()
  {
    differentialDrive.close();
    leftEncoder.close();
    rightEncoder.close();
    leftLeader.close();
    rightLeader.close();
    gyroAhrs.close();
    field.close();
  }
}
