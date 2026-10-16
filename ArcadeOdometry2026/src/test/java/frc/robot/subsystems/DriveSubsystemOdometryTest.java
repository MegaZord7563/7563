// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OdometryConstants;
import frc.robot.Constants.SimConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Exercises odometry against the simulated drivetrain. The simulation is the only reference
 * available off-robot, so every test here compares the estimated pose with the model's own pose:
 * the two agreeing means the encoders, the heading sign and the geometry constants line up.
 */
class DriveSubsystemOdometryTest
{
  private DriveSubsystem drive;

  @BeforeEach
  void setUp()
  {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();

    drive = new DriveSubsystem();
  }

  @AfterEach
  void tearDown()
  {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    drive.close();
    HAL.shutdown();
  }

  /**
   * Runs the subsystem for the given time the way the CommandScheduler does -- periodic()
   * first, then simulationPeriodic() -- and then brings the robot to rest.
   *
   * <p>The settling matters. Because periodic() runs before simulationPeriodic(), the estimate
   * always trails the model by one 20 ms step; while spinning at 165 deg/s that is over 3 deg
   * of pure lag, which would swamp any genuine odometry error being measured. Coasting to a
   * stop and then letting odometry read the final state removes the lag without hiding error.
   */
  private void run(double seconds, double forward, double rotation)
  {
    for (int i = 0; i < (int) Math.round(seconds / 0.020); i++)
    {
      drive.driveMotor(forward, rotation);
      drive.periodic();
      drive.simulationPeriodic();
    }

    for (int i = 0; i < 25; i++) // 0.5 s of coasting
    {
      drive.driveMotor(0.0, 0.0);
      drive.periodic();
      drive.simulationPeriodic();
    }

    drive.periodic(); // catch the estimate up to the model's final state
  }


  @Test
  void startsAtTheConfiguredPose()
  {
    assertEquals(FieldConstants.kStartingPose.getX(), drive.getPose().getX(), 1e-6);
    assertEquals(FieldConstants.kStartingPose.getY(), drive.getPose().getY(), 1e-6);
    assertEquals(0.0, drive.getLeftDistanceMeters(), 1e-9);
    assertEquals(0.0, drive.getRightDistanceMeters(), 1e-9);
  }

  @Test
  void distancePerPulseConvertsCountsToMetres()
  {
    // One full wheel revolution of counts must equal one wheel circumference of travel.
    double circumference = Math.PI * OdometryConstants.kWheelDiameterMeters;
    assertEquals(circumference,
                 OdometryConstants.kEncoderCountsPerWheelRevolution
                     * OdometryConstants.kEncoderDistancePerPulse,
                 1e-9);
  }

  @Test
  void simulatedWheelSizeMatchesTheConfiguredWheelSize()
  {
    // Simulation CANNOT catch a wrong kWheelDiameterMeters. EncoderSim.setDistance() converts
    // metres to counts with the very distance-per-pulse that getDistance() converts back with,
    // so the scaling cancels and the pose comes out right even when the constant is wrong --
    // verified by setting it to 8 in, which leaves every other test in this class passing.
    //
    // So the constant has to be measured on the real robot. What this test can do is stop the
    // two halves drifting apart: the drivetrain model has its own wheel size, and if someone
    // changes SimConstants.kWheelSize without changing kWheelDiameterMeters, every simulated
    // distance silently stops meaning what the odometry constants say it means.
    assertEquals(SimConstants.kWheelSize.value, OdometryConstants.kWheelDiameterMeters, 1e-9,
                 "SimConstants.kWheelSize and OdometryConstants.kWheelDiameterMeters disagree");
  }

  @Test
  void drivingForwardMovesThePoseForwardAndTracksTheModel()
  {
    Pose2d start = drive.getPose();

    run(2.0, 0.6, 0.0);

    Pose2d pose = drive.getPose();

    // Both encoders must read positive and very nearly equal going straight.
    assertTrue(drive.getLeftDistanceMeters() > 0.1,
               "left encoder did not advance: " + drive.getLeftDistanceMeters());
    assertEquals(drive.getLeftDistanceMeters(), drive.getRightDistanceMeters(), 0.02);

    // Heading 0 means +X is forward, so the robot should have moved in +X only.
    assertTrue(pose.getX() - start.getX() > 0.1,
               "pose did not advance in +X: " + pose.getX());
    assertEquals(start.getY(), pose.getY(), 0.05);
    assertEquals(0.0, pose.getRotation().getDegrees(), 2.0);

    // The estimate must agree with the model that generated the sensor values.
    assertPoseTracksModel(0.05);

    // Distance driven should match the average wheel travel.
    assertEquals(drive.getAverageDistanceMeters(), pose.getX() - start.getX(), 0.05);
  }

  @Test
  void turningLeftIncreasesTheHeadingAndTracksTheModel()
  {
    // Positive rotation into arcadeDrive turns the robot counter-clockwise, which must read
    // as an INCREASING pose heading. If the gyro sign were wrong this is the test that fails.
    run(1.0, 0.0, 0.5);

    assertTrue(drive.getPose().getRotation().getDegrees() > 5.0,
               "turning left did not increase the pose heading: "
                   + drive.getPose().getRotation().getDegrees());

    // Turning also curves the translation, so this checks the pose is integrated along the
    // heading rather than just accumulating straight-line distance.
    assertPoseTracksModel(0.10);
  }

  @Test
  void resetOdometryMovesTheEstimateAndKeepsTracking()
  {
    run(1.0, 0.6, 0.0);

    Pose2d declared = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(90.0));
    drive.resetOdometry(declared);

    assertEquals(3.0, drive.getPose().getX(), 1e-6);
    assertEquals(4.0, drive.getPose().getY(), 1e-6);
    assertEquals(90.0, drive.getPose().getRotation().getDegrees(), 1e-6);
    assertEquals(0.0, drive.getLeftDistanceMeters(), 1e-9);

    // Driving on from the declared pose must now go along +Y, since the robot faces 90 deg.
    run(1.0, 0.6, 0.0);
    assertTrue(drive.getPose().getY() > 4.1,
               "did not continue along the declared heading: " + drive.getPose().getY());
    assertEquals(3.0, drive.getPose().getX(), 0.05);
  }

  @Test
  void resetGyroLeavesThePoseWhereItWas()
  {
    run(1.0, 0.4, 0.4); // drive a curve, so both position and heading are non-trivial

    Pose2d before = drive.getPose();
    drive.resetGyro();
    Pose2d after = drive.getPose();

    assertEquals(before.getX(), after.getX(), 1e-6);
    assertEquals(before.getY(), after.getY(), 1e-6);
    assertEquals(before.getRotation().getDegrees(), after.getRotation().getDegrees(), 1e-6);

    // The heading reference is zeroed for drive2Heading, even though the pose did not move.
    assertEquals(0.0, drive.getYaw(), 1e-6);

    // And odometry keeps working across the reset rather than jumping.
    run(1.0, 0.4, 0.0);
    assertPoseTracksModel(0.10);
  }

  @Test
  void resetEncodersDoesNotMoveThePose()
  {
    run(1.0, 0.6, 0.0);

    Pose2d before = drive.getPose();
    drive.resetEncoders();

    assertEquals(0.0, drive.getLeftDistanceMeters(), 1e-9);
    assertEquals(0.0, drive.getRightDistanceMeters(), 1e-9);
    assertEquals(before.getX(), drive.getPose().getX(), 1e-6);
    assertEquals(before.getY(), drive.getPose().getY(), 1e-6);

    run(1.0, 0.6, 0.0);
    assertTrue(drive.getPose().getX() > before.getX() + 0.1,
               "odometry stopped advancing after resetEncoders()");
  }

  /** Asserts the odometry estimate still agrees with the simulation model that drives it. */
  private void assertPoseTracksModel(double toleranceMeters)
  {
    Pose2d estimate = drive.getPose();
    Pose2d truth = drive.getSimulatedPose();

    assertEquals(truth.getX(), estimate.getX(), toleranceMeters,
                 "odometry X drifted from the simulated truth");
    assertEquals(truth.getY(), estimate.getY(), toleranceMeters,
                 "odometry Y drifted from the simulated truth");
    assertEquals(truth.getRotation().getDegrees(), estimate.getRotation().getDegrees(), 2.0,
                 "odometry heading drifted from the simulated truth");
  }
}
