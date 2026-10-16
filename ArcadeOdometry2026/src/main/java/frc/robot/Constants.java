// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
  public static class DriveConstants 
  {
    public static final int kLeftMotorPort = 4;
    public static final int kRightMotorPort = 3;
    
    public static final IdleMode kIdleModeLeftMotor = IdleMode.kBrake;
    public static final IdleMode kIdleModeRightMotor = IdleMode.kBrake;

    public static final int currentLimit = 40; // Amps

    // Heading (turn-in-place) control
    public static final double kHeadingToleranceDeg = 5.0;
    public static final double kHeadingRateToleranceDegPerSec = 10.0;
    /** Maximum turn output applied by drive2Heading. Lower this to reduce overshoot. */
    public static final double kMaxTurnOutput = 0.6;
  }

  /**
   * Encoders and drivetrain geometry, which together define odometry.
   *
   * <p>The SPARK MAXes run brushed motors, so they expose no built-in encoder. Odometry therefore
   * reads quadrature encoders wired to the roboRIO DIO ports.
   *
   * <p>IMPORTANT: every number here except the DIO channels must be MEASURED on the real robot.
   * The defaults deliberately match the kitbot model in {@link SimConstants} so that simulation is
   * self-consistent, but odometry on the real robot is only ever as accurate as these values.
   */
  public static class OdometryConstants
  {
    public static final int kLeftEncoderChannelA = 0;
    public static final int kLeftEncoderChannelB = 1;
    public static final int kRightEncoderChannelA = 2;
    public static final int kRightEncoderChannelB = 3;

    /**
     * Flip a side if it reports a NEGATIVE distance while the robot drives FORWARD. Both sides
     * must read positive going forward, otherwise odometry sees driving straight as a turn.
     */
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    /** Wheel diameter. Measure a worn wheel, not the catalogue value. */
    public static final double kWheelDiameterMeters = Units.inchesToMeters(6.0);

    /**
     * Distance between the left and right wheel contact patches. Note that this does NOT affect
     * the odometry pose: DifferentialDriveOdometry takes its heading from the gyro and its
     * distance from the average of the two sides, so neither one uses the track width. It is the
     * kinematics model -- chassis speeds to and from wheel speeds, and any future path following
     * -- that depends on it. 26 in is the value DifferentialDrivetrainSim.createKitbotSim models.
     */
    public static final double kTrackWidthMeters = Units.inchesToMeters(26.0);

    /** Encoder cycles (not edges) per revolution of the encoder shaft. 2048 = REV Through Bore. */
    public static final double kEncoderCyclesPerRevolution = 2048.0;

    /** Encoder shaft revolutions per wheel revolution. 1.0 when mounted on the wheel shaft. */
    public static final double kEncoderGearRatio = 1.0;

    /** k4X decoding counts all four edges of every cycle. */
    public static final double kEncoderCountsPerWheelRevolution =
        kEncoderCyclesPerRevolution * 4.0 * kEncoderGearRatio;

    /** Metres of wheel travel per encoder count, which is what Encoder.getDistance() scales by. */
    public static final double kEncoderDistancePerPulse =
        (Math.PI * kWheelDiameterMeters) / kEncoderCountsPerWheelRevolution;
  }

  /** 2026 Rebuilt field geometry, used to place the robot and to seed odometry. */
  public static class FieldConstants
  {
    public static final double kFieldLengthMeters = 16.541;
    public static final double kFieldWidthMeters = 8.069;

    /**
     * Pose odometry starts from. Mid-field, so the robot is on screen in the Sim GUI instead of
     * in the bottom-left corner. Replace with the real starting pose for a match.
     */
    public static final Pose2d kStartingPose =
        new Pose2d(kFieldLengthMeters / 2.0, kFieldWidthMeters / 2.0, new Rotation2d());
  }

  /**
   * Simulation-only drivetrain model. These are KITBOT DEFAULTS, not measured values for
   * this robot -- correct them to match the real drivetrain or the simulated motion will
   * not resemble what the robot actually does.
   */
  public static class SimConstants
  {
    public static final KitbotMotor kMotor = KitbotMotor.kDualCIMPerSide;
    public static final KitbotGearing kGearing = KitbotGearing.k10p71;
    public static final KitbotWheelSize kWheelSize = KitbotWheelSize.kSixInch;
  }
  public static class OperatorConstants 
  {
    public static final int kDriverControllerPort = 0;
  }
}
