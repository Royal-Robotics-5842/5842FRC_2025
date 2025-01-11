// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SetSwerveSpeed extends Command {

  private final SwerveSubsystem swerve;

  public SetSwerveSpeed(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute(){

    //This is making the swervemodule drive at a constant speed
    swerve.frontLeft.setSpeedDrive(0.3);
    swerve.frontRight.setSpeedDrive(0.3);
    swerve.backLeft.setSpeedDrive(0.3);
    swerve.backRight.setSpeedDrive(0.3);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
