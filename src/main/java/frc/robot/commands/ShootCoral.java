// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralShooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCoral extends Command {
  /** Creates a new ShootCoral. */
  public CoralShooter coral;
  public double speed;
  public double startTime;
  public ShootCoral(CoralShooter coral, double speed) {
    this.coral = coral;
    this.speed = speed;
    addRequirements(coral);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    startTime = coral.coralTimer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    coral.leftMotor.set(speed);
    coral.rightMotor.set(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    coral.leftMotor.set(0);
    coral.rightMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
    if (coral.coralTimer.getFPGATimestamp() - startTime >= 0.5 && (coral.leftMotor.getOutputCurrent() >=20))
    {
      return true;
    }
    */
    return false;
  }
}
