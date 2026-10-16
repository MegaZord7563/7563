// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
