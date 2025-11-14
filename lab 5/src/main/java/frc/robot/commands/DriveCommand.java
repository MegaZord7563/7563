// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubSystem;

public class DriveCommand extends Command 
{
  public DriveSubSystem drive = RobotContainer.drive;
  
  
  /** Creates a new DriveCommand. */
  public DriveCommand() 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

    drive.resetEncoder();
    
    System.out.println("Drivecommand iniciado");
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  { 
    
    drive.driveToVelocity(SmartDashboard.getNumber("Motor/Velocity", 0));
   
    
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    
    drive.stopMotor();
   
    System.out.println("Drivecommand encerrado");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {

    return RobotContainer.m_driverController.getHID().getAButton();
  }
}
