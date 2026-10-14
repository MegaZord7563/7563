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

  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() 
  {
    leftLeader = new SparkMax(DriveConstants.kLeftMotorPort, MotorType.kBrushed);
    //leftFollower = new SparkMax(2, MotorType.kBrushed);
    rightLeader = new SparkMax(DriveConstants.kRightMotorPort, MotorType.kBrushed);
  }

    /*
     * Create new SPARK MAX configuration objects. These will store the
     * configuration parameters for the SPARK MAXes that we will set below.
     */
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
}
