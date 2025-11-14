// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubSystem extends SubsystemBase 
{

  private final CANSparkMax motorSpark; 
  private final RelativeEncoder motorEncoder;
  
  /** Creates a new DriveSubSystem. */
  public DriveSubSystem() 
  {
    motorSpark = new CANSparkMax(Constants.motorSparkId,MotorType.kBrushless);

    motorEncoder = motorSpark.getEncoder();


    //motorSpark settings

    motorSpark.restoreFactoryDefaults();
    motorSpark.setIdleMode(Constants.kMotorIdleMode);
    motorSpark.setSmartCurrentLimit(Constants.kMotorCurrentLimit);
    //motorRelativeEncoder.setPositionConversionFactor(1);
    //motorRelativeEncoder.setVelocityConversionFactor(1);
    
    motorSpark.burnFlash();


  }
  /**
   * 
   * @param speed
   */
  public void  driveMotor(double speed)
  {
    motorSpark.set(speed);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    /**
     * Encoder position is read from a RelativeEncoder object by calling the
     * GetPosition() method.
     * 
     * GetPosition() returns the position of the encoder in units of revolutions
     */
    SmartDashboard.putNumber("Encoder Position", motorEncoder.getPosition());

    /**
     * Encoder velocity is read from a RelativeEncoder object by calling the
     * GetVelocity() method.
     * 
     * GetVelocity() returns the velocity of the encoder in units of RPM
     */
    SmartDashboard.putNumber("Encoder Velocity", motorEncoder.getVelocity());
  }


}
