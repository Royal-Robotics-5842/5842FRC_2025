// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants.Side;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Positoning extends Command {
  /** Creates a new Positoning. */
  SwerveSubsystem swerve;
  ChassisSpeeds speeds;
  Pose2d finalPose = new Pose2d();
  Pose2d offsetPosition = new Pose2d();
  Side side;
  public Positoning(SwerveSubsystem swerve, Side side) {
    this.swerve = swerve;
    this.side = side;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    if(swerve.allianceColor.toString() == "Red")
    {
      switch (((int)LimelightHelpers.getFiducialID("limelight-lite"))) {
        case 6:
          finalPose = new Pose2d(1,1,Rotation2d.fromDegrees(0.0));
          break;
        case 7:
          finalPose = new Pose2d(1,1,Rotation2d.fromDegrees(0.0));
          break;
        case 8:
          finalPose = new Pose2d(1,1,Rotation2d.fromDegrees(0.0));
          break;
        case 9:
          finalPose = new Pose2d(1,1,Rotation2d.fromDegrees(0.0));
          break;
        case 10:
          finalPose = new Pose2d(1,1,Rotation2d.fromDegrees(0.0));
          break;
        case 11:
          finalPose = new Pose2d(1,1,Rotation2d.fromDegrees(0.0));
          break;
      }

    }
    else if (swerve.allianceColor.toString() == "Blue") {
      switch (((int)LimelightHelpers.getFiducialID("limelight-lite"))) {
        case -1:
          finalPose = swerve.lastPose;
          break;
        case 17:
          finalPose = new Pose2d(4.048, 3.251, Rotation2d.fromDegrees(60));
          break;
        case 18:
          finalPose = new Pose2d(3.7,4.020,Rotation2d.fromDegrees(0.0));
          break;
        case 19:
          finalPose = new Pose2d(4.1, 4.83, Rotation2d.fromDegrees(-60));
          break;
        case 20:
          finalPose = new Pose2d(5.07,4.8,Rotation2d.fromDegrees(-120));
          break;
        case 21:
          finalPose = new Pose2d(5.4,4.02,Rotation2d.fromDegrees(-180));
          break;
        case 22:
          finalPose = new Pose2d(4.967,3.241,Rotation2d.fromDegrees(120));
          break;
      }
    }

    switch (side) {
      case left:
        offsetPosition = new Pose2d(
        (finalPose.getX()) - 
        (Constants.AutoConstants.reef_offset_left * (Math.sin(finalPose.getRotation().getRadians()))), 
        
        finalPose.getY() + 
        (Constants.AutoConstants.reef_offset_left* Math.cos(finalPose.getRotation().getRadians())), 
        
        finalPose.getRotation());
        break;
      case right:
      offsetPosition = new Pose2d(
        (finalPose.getX()) + 
        (Constants.AutoConstants.reef_offset_left * (Math.sin(finalPose.getRotation().getRadians()))), 
        
        finalPose.getY() - 
        (Constants.AutoConstants.reef_offset_left* Math.cos(finalPose.getRotation().getRadians())), 
        
        finalPose.getRotation());
        
        break;
      case middle:
      offsetPosition = new Pose2d(finalPose.getX(),finalPose.getY(), Rotation2d.fromDegrees(0));
        break;
    }
    
    swerve.setLastPose(finalPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    speeds = swerve.holo.calculate(swerve.getLimelightPose2d(), 
    offsetPosition, 0, offsetPosition.getRotation());
    
    swerve.setModuleStates(Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));

    System.out.println("FINAL POSE" + finalPose.toString());
    System.out.println("Offeset 1" + offsetPosition.toString());
    System.out.println("Rotation" + finalPose.getRotation().getDegrees());
    SmartDashboard.putString("OffsetPose", offsetPosition.toString());
    //swerve.setLastPose(finalPose);
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
