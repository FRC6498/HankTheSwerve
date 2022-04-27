// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveConstants;

public class SwerveDrive extends SubsystemBase {
  public SwerveDriveOdometry odometry;
  public SwerveModule[] swerveModules;
  public AHRS navx;
  /** Creates a new ExampleSubsystem. */
  public SwerveDrive() {
    navx = new AHRS(Port.kMXP);

    odometry = new SwerveDriveOdometry(DriveConstants.swerveKinematics, getYaw());
    swerveModules = new SwerveModule[] {
      new SwerveModule(0, DriveConstants.Mod0.constants),
      new SwerveModule(1, DriveConstants.Mod1.constants),
      new SwerveModule(2, DriveConstants.Mod2.constants),
      new SwerveModule(3, DriveConstants.Mod3.constants)
    };
  }

  public void zeroGyro() {
    navx.reset();
  }

  public Rotation2d getYaw() {
    return navx.getRotation2d();
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    for (SwerveModule mod : swerveModules) {
      moduleStates[mod.moduleNumber] = mod.getState();
    }
    return moduleStates;
  }

  // for open loop driving
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    // determine desired states
    SwerveModuleState[] states = DriveConstants.swerveKinematics.toSwerveModuleStates(
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds( // provided translation is field relative
        translation.getX(), translation.getY(), rotation, getYaw()
      )
      :
      new ChassisSpeeds( // provided translation is robot relative
        translation.getX(), 
        translation.getY(), 
        rotation
      )
    );

    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxSpeed);
  
    for (SwerveModule mod : swerveModules) {
      mod.setDesiredState(states[mod.moduleNumber], isOpenLoop);
    }
  }

  // for closed loop driving
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.maxSpeed);
  
    for (SwerveModule mod : swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, getYaw());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(getYaw(), getModuleStates());
  }
}
