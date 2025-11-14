// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubSystem;

public class DriveCommand extends Command 
{
  public DriveSubSystem drive = RobotContainer.drive;
  //PID constants
  private final double KP=0.2000;
  private final double KI=0.0;
  private final double KD=0.0;
  
  //Trapezoidal constants
  private static double kDt = 0.02; //The period between controller updates in seconds. 
  private static double kMaxVelocity = 4;
  private static double kMaxAcceleration = 1;
  

  private final TrapezoidProfile.Constraints m_constraints =  new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
  private final ProfiledPIDController mController = new ProfiledPIDController(KP, KI, KD, m_constraints,kDt);
     

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
    mController.reset(0,0);
    //mController.setConstraints(m_constraints);
    //mController.setTolerance(1);
    System.out.println("Drivecommand iniciado");
    //SmartDashboard.putNumber("ValorP", 0);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  { 
    
    //SmartDashboard.getEntry("ValorP");
    mController.setP(SmartDashboard.getNumber("Parametro PID/Valor P",0));
    
    double outPut = mController.calculate(drive.getMotorPosition(), (SmartDashboard.getNumber("Parametro Movimento/ SetPoint Voltas",0)));
    drive.driveMotor(outPut);

    SmartDashboard.putNumber("PID output", outPut);
    
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
