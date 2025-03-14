// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.pivot;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.driveSubsytem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
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
  private final driveSubsytem drive = new driveSubsytem();
  private final pivot pivot = new pivot();
  private final intake intake = new intake();



  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller joystick =
      new CommandPS4Controller(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
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
  private void configureBindings() {

    //Spin a Thing

    //Move a Thing to a certain place
    pivot.setDefaultCommand(pivot.setVelocityCommand(0));
    joystick.circle().whileTrue(pivot.setPositionCommand(0));
    joystick.triangle().whileTrue(pivot.setPositionCommand(100));

     //Move a Thing to a certain place
     intake.setDefaultCommand(intake.setVelocityCommand(0));
     joystick.cross().whileTrue(intake.setVelocityCommand(1));
     joystick.square().whileTrue(intake.setVelocityCommand(-1));
    
    //Arcade Drive
    drive.setDefaultCommand(drive.Arcadedrive(joystick.getLeftY(),joystick.getRightX()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new SequentialCommandGroup(drive.Arcadedrive(1, 0).withTimeout(0.5),pivot.setPositionCommand(0).withTimeout(1),intake.outtakeCommand().withTimeout(1));
  }
}
