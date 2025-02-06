// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

public class resetEverything extends Command {
  private final SwerveSubsystem swerve;
  private final Elevator elevator;
  public resetEverything(SwerveSubsystem swerve, Elevator elevator) {
    this.swerve = swerve;
    this.elevator = elevator;
    addRequirements(swerve, elevator);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    System.out.println("resetEverything START");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    swerve.zeroHeading();

    swerve.frontLeft.setToAngle(0);
    swerve.frontRight.setToAngle(0);
    swerve.backLeft.setToAngle(0);
    swerve.backRight.setToAngle(0);

    elevator.leftMotor.getEncoder().setPosition(0);
/*
    swerve.frontLeft.setDrivePosition(0);
    swerve.frontRight.setDrivePosition(0);
    swerve.backLeft.setDrivePosition(0);
    swerve.backRight.setDrivePosition(0);
*/
    swerve.resetOdometry(new Pose2d(0,0, swerve.gyro.getRotation2d()));
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("resetEverything ENDED");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
