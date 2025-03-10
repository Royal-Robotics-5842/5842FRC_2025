// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class elevPID extends Command {
  /** Creates a new elevPID. */
  private Elevator elevator;
  private double position;
  public elevPID(Elevator elevator, double position) {
    this.elevator = elevator;
    this.position = position;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    //Top
    if (elevator.getPosition()< position)
    {
      elevator.leftMotor.set(0.01);
      elevator.rightMotor.set(0.01);
    }

    //Bottom
    if (elevator.getPosition() > position)
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
    elevator.moveElevator(position);
    if(elevator.getBottomLimit() == false)
    {
      elevator.leftMotor.getEncoder().setPosition(0);
    }
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
    if((elevator.getTopLimit() == false && elevator.leftMotor.getAppliedOutput() > 0))
    {
      return true;
    }

    if((elevator.getBottomLimit() == false && elevator.leftMotor.getAppliedOutput() < 0))
    {
      return true;
    }

    return false;
  }
}
