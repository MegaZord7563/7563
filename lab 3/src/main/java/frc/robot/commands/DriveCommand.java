// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubSystem;

public class DriveCommand extends Command 
{
  public DriveSubSystem drive = RobotContainer.drive;
  private final double KP=0.2000;
  private final double KI=0.0;
  private final double KD=0.0;
  public PIDController mController = new PIDController(KP, KI, KD);

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
    mController.reset();
    mController.setTolerance(1);
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
    mController.close();
    drive.stopMotor();
   
    System.out.println("Drivecommand encerrado");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {

    return RobotContainer.m_driverController.getHID().getAButton();//|| mController.atSetpoint();
  }
}
