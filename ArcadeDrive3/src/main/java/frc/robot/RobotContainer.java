// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static DriveSubsystem driveSubsystem = new DriveSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverJoystick =
                                    new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    // Configure the trigger bindings
    configureBindings();

    //Runnable command
    driveSubsystem.setDefaultCommand
                                    (new RunCommand(
                                                    () -> driveSubsystem.driveMotor
                                                      ( MathUtil.applyDeadband((driverJoystick.getLeftY()*0.6),0.1), 
                                                        MathUtil.applyDeadband((driverJoystick.getRightX()*0.6),0.1)
                                                      ), 
                                                      driveSubsystem
                                                    )
                                    );
                                  
  

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() 
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    driverJoystick.b().whileTrue(new RunCommand(()-> driveSubsystem.drive2Heading
                                                      (SmartDashboard.getNumber("Heading SetPoint", 0)),
                                                      driveSubsystem).until(()-> driveSubsystem.pidAtSetPoint()));
    
    // Reset the gyro when the START button is pressed
    driverJoystick.start().whileTrue(new InstantCommand(driveSubsystem::resetGyro));
  
  
  }
                                      

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() 
  {
    // An example command will be run in autonomous
    return 
    new SequentialCommandGroup( //Reset gyro
                                new InstantCommand(driveSubsystem::resetGyro),
                                
                                // Turn to 90 degrees                                           
                                new RunCommand(()-> driveSubsystem.drive2Heading(90), driveSubsystem)
                                                .until(()->driveSubsystem.pidAtSetPoint()), // Wait until the robot reaches 90 degrees,
                                //new WaitCommand(1) ,                                     
                                
                                new RunCommand(()-> driveSubsystem.drive2Heading(180), driveSubsystem)
                                                .until(()->driveSubsystem.pidAtSetPoint()) // Wait until the robot reaches 90 degrees,
                                
                                                //new WaitCommand(1)   ,  
                                // Turn to 90 degrees                                           
                                //new RunCommand(()-> driveSubsystem.drive2Heading(270), driveSubsystem)
                                               // .until(()->driveSubsystem.pidAtSetPoint()),
                                
                               // new RunCommand(()-> driveSubsystem.drive2Heading(180), driveSubsystem)
                                               // .until(()->driveSubsystem.pidAtSetPoint()) // Wait until the robot reaches 90 degrees,
                

                                

                             
                              );
             
                         
  }
}
