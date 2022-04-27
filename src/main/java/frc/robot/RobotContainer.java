// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.OI.CommandXboxController;
import frc.robot.commands.SwerveDriveOpenLoop;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  boolean fieldRelative;
  boolean openLoop;
  CommandXboxController driver = new CommandXboxController(0);

  // The robot's subsystems and commands are defined here...
  private final SwerveDrive drivetrain = new SwerveDrive();

  private final SwerveDriveOpenLoop m_autoCommand = new SwerveDriveOpenLoop(drivetrain, driver::getLeftY, driver::getLeftX, driver::getRightX, true, true);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    fieldRelative = true;
    openLoop = true;
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // zero gyro
    driver.y().whenPressed(new InstantCommand(() -> drivetrain.zeroGyro()));
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
