// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase 
{
  private final SparkMax leftLeader;
  //SparkMax leftFollower;
  private final SparkMax rightLeader;
  //SparkMax rightFollower;
  private final DifferentialDrive differentialDrive;
  // Constants for PID control


 
  private double kP = 0.0022;//0.01//0.00785
  private double kP1 = 0.0180;//0.00795

  PIDController pidController = new PIDController(kP1, 0.0, 0.00); // Initialize PID controller with kP, kI, kD


  //private final Encoder leftEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
  private final AHRS gyroAhrs;// = new AHRS(NavXComType.kMXP_SPI,AHRS.NavXUpdateRate.k50Hz); // Adjusted to use a valid constructor

  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() 
  {
    leftLeader = new SparkMax(DriveConstants.kLeftMotorPort, MotorType.kBrushed);
    //leftFollower = new SparkMax(2, MotorType.kBrushed);
    rightLeader = new SparkMax(DriveConstants.kRightMotorPort, MotorType.kBrushed);
    //rightFollower = new SparkMax(4, MotorType.kBrushless);
    gyroAhrs = new AHRS(NavXComType.kMXP_SPI,AHRS.NavXUpdateRate.k50Hz);

    // Initialize the DifferentialDrive with the left and right leaders 
    differentialDrive = new DifferentialDrive(leftLeader, rightLeader);

    pidController.setTolerance(5.0); // Set the tolerance for the PID controller

    
  

    SmartDashboard.putNumber("Heading SetPoint", 0.0);
    SmartDashboard.putNumber("KP Heading",0 );
    

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
    
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run

    //kP1=SmartDashboard.getNumber("KP Heading",0 );
    SmartDashboard.putNumber("Current Heading", this.getHeading());
    SmartDashboard.putNumber("Gyro Yaw",gyroAhrs.getYaw()); // Update the yaw value to keep it fresh
    SmartDashboard.putNumber("KP Heading",kP1);
    
    
  }
  

  /**
   * Drives the motors based on the forward and rotation values.
   *
   * @param forward The forward speed, where positive is forward and negative is backward.
   * @param rotation The rotation speed, where positive is clockwise and negative is counterclockwise.
   */
  public void driveMotor(double forward, double rotation) 
  {
    // This method drives the motors based on the forward and rotation values
    // Get the current rate of the gyro
    double error = -this.getRate(); 
    double kForward = forward + error * kP; // Adjust forward speed based on gyro rate
    double kRotation = rotation - error * kP; // Adjust rotation speed based on gyro rate
    
    // Use the error to adjust the forward and rotation values
    // This is a simple proportional control to adjust the drive based on gyro rate
    // You can adjust the kP value to change the responsiveness of the control
    // For example, if the robot is turning too fast, you can reduce the forward speed
    
    //differentialDrive.tankDrive(kForward, kRotation); // Use arcade drive for simpler control

    differentialDrive.arcadeDrive(forward , rotation);
  }

  public boolean pidAtSetPoint()
  {
    // This method checks if the PID controller is within the tolerance
    // It can be used to determine if the robot has reached the target heading
    if (pidController.atSetpoint()) 
    {
      return true;
    } 
    else 
    {
      return false;
    }
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
    // This method is intended to drive the robot to a specific heading
    // Implementation would depend on the desired behavior, such as using PID control
    // to adjust the motors based on the current heading and target heading.
    
    double targetHeading = heading; // Set your target heading here
    double currentHeading = this.getHeading(); // Get the current heading of the robot
    //double error = targetHeading - currentHeading; // Calculate the error between target and current heading
    //double rotation = error * kP1; // Calculate the rotation speed based on the error
    // Set the speed of the motors based on the forward speed and rotation
    //differentialDrive.tankDrive(-rotation*0.8, rotation*0.8);
    pidController.setSetpoint(targetHeading); // Set the target heading for the PID controller
    double output = pidController.calculate(currentHeading); // Calculate the output based on the current heading
    differentialDrive.tankDrive(-output, output); // Drive the robot towards the target heading   

    
  }

  /**
   * Gets the current heading of the robot.
   * @return The current heading in degrees.
   */
  public double getHeading() 
  {
    // Returns the current heading of the robot
    return gyroAhrs.getAngle();
  } 
  /**
   * Gets the current yaw of the robot.
   * @return The current yaw in degrees.
   */ 
  public double getYaw() 
  {
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
  }
}
