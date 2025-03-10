// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDriveJoystick extends Command {
  
  private final SwerveSubsystem swerveSubsystem;
  private final Elevator elevator;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;

  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  public SwerveDriveJoystick(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, 
                            Supplier<Boolean> fieldOrientedFunction, Elevator elevator) { 
              this.swerveSubsystem = swerveSubsystem;
              this.xSpdFunction = xSpdFunction;
              this.ySpdFunction = ySpdFunction;
              this.turningSpdFunction = turningSpdFunction;
              this.fieldOrientedFunction = fieldOrientedFunction;
              this.elevator = elevator;

              this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
              this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
              this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(swerveSubsystem);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    // 1. Get real-time joystick inputs
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();

    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
    
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed)
            * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get())
        {
          if (elevator.getPosition() > Constants.elevatorConstants.L4_height/2)
          {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed/2, ySpeed/2, turningSpeed/2, swerveSubsystem.getRotation2d());
          }
          else
          {chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
          }
        }
         else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }
        
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    
    // 6. Output each module states to wheels
    swerveSubsystem.setModuleStates(moduleStates);

  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(RobotContainer.driverJoytick.getRawAxis(OIConstants.kDriverYAxis)) <= 0.1 && 
           Math.abs(RobotContainer.driverJoytick.getRawAxis(OIConstants.kDriverXAxis)) <= 0.1;
  }
}
