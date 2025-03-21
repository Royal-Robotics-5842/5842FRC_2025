// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class moveElevator extends Command {
  /** Creates a new moveElevator. */
  private Elevator elevator;
  private double speed;
  public moveElevator(Elevator elevator, double speed) {
    this.elevator = elevator;
    this.speed = speed;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    if (speed > 0)
    {
      elevator.leftMotor.set(0.01);
      elevator.rightMotor.set(0.01);
    }

    if (speed < 0 )
    {
      elevator.leftMotor.set(-0.01);
      elevator.rightMotor.set(-0.01);
    }
    System.out.println("Elevator up/down Started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    elevator.leftMotor.set(speed);
    elevator.rightMotor.set(speed);

    System.out.println("ENCODER" + elevator.rightMotor.getEncoder().getPosition());

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    elevator.leftMotor.set(0);
    elevator.rightMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((elevator.getTopLimit() == false && elevator.rightMotor.getAppliedOutput() > 0))
    {
      return true;
    }

    if((elevator.getBottomLimit() == false && elevator.rightMotor.getAppliedOutput() < 0))
    {
      return true;
    }
    return false;
  }
}
