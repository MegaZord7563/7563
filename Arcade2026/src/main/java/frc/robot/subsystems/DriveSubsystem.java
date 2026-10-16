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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SimConstants;

public class DriveSubsystem extends SubsystemBase
{
  private final SparkMax leftLeader;
  //SparkMax leftFollower;
  private final SparkMax rightLeader;
  //SparkMax rightFollower;
  private final DifferentialDrive differentialDrive;
  // Constants for PID control



  private double kP1 = 0.0180;//0.00795

  PIDController pidController = new PIDController(kP1, 0.0, 0.00); // Initialize PID controller with kP, kI, kD


  //private final Encoder leftEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
  private final AHRS gyroAhrs;// = new AHRS(NavXComType.kMXP_SPI,AHRS.NavXUpdateRate.k50Hz); // Adjusted to use a valid constructor

  // --- Simulation ---------------------------------------------------------------------
  // The NavX has no simulation model (the Studica AHRS registers no SimDevice), and the
  // brushed SPARK MAXes have no encoders, so there is no real sensor to simulate from.
  // DifferentialDrivetrainSim models the drivetrain from motor voltages instead, which
  // gives both a pose for the Field2d widget and a usable heading for drive2Heading.
  private final DifferentialDrivetrainSim driveSim =
      DifferentialDrivetrainSim.createKitbotSim(SimConstants.kMotor,
                                                SimConstants.kGearing,
                                                SimConstants.kWheelSize,
                                                null); // null = no measurement noise
  private final Field2d field = new Field2d();
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

    if (RobotBase.isSimulation())
    {
      // Only publish the field in simulation. On the real robot there is no odometry, so
      // the widget would just show the robot parked at the origin, which is misleading.
      // Select "2026 Rebuilt" as the builtin image in the Sim GUI's Field2d window.
      SmartDashboard.putData("Field", field);

      // Start mid-field (the 2026 field is 16.541 m x 8.069 m) so the robot is on screen
      // instead of in the bottom-left corner.
      driveSim.setPose(new Pose2d(16.541 / 2.0, 8.069 / 2.0, new Rotation2d()));
    }

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

    SmartDashboard.putNumber("Current Heading", this.getHeading());
    SmartDashboard.putNumber("Gyro Yaw", this.getYaw()); // Update the yaw value to keep it fresh


  }

  /**
   * Advances the simulated drivetrain. Called by the CommandScheduler only in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
    // DifferentialDrive drives both sides positive for forward and relies on the right
    // SPARK's own inversion, so get() already reports "positive = forward" for both.
    double batteryVolts = RobotController.getBatteryVoltage();
    driveSim.setInputs(leftLeader.get() * batteryVolts, rightLeader.get() * batteryVolts);

    driveSim.update(0.020); // 20 ms, matching the default TimedRobot period

    field.setRobotPose(driveSim.getPose());
  }


  /**
   * Drives the motors based on the forward and rotation values.
   *
   * @param forward The forward speed, where positive is forward and negative is backward.
   * @param rotation The rotation speed, where positive is clockwise and negative is counterclockwise.
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
  }
}
