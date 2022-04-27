// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SwerveDriveOpenLoop extends CommandBase {
  private double rotation;
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;
  private final SwerveDrive drivetrain;

  private DoubleSupplier forward;
  private DoubleSupplier strafe;
  private DoubleSupplier turnSupplier;

  /**
   * Creates a new ExampleCommand.
   *
   * @param drivetrain The subsystem used by this command.
   */
  public SwerveDriveOpenLoop(SwerveDrive drivetrain, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turnSupplier, boolean fieldRelative, boolean openLoop) {
    this.drivetrain = drivetrain;
    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;
    this.forward = forward;
    this.strafe = strafe;
    this.turnSupplier = turnSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    translation = new Translation2d(forward.getAsDouble(), strafe.getAsDouble()).times(DriveConstants.maxSpeed);
    rotation = turnSupplier.getAsDouble() * DriveConstants.maxAngularVelocity;
    drivetrain.drive(translation, rotation, fieldRelative, openLoop);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
