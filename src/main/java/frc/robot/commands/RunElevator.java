package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSystem;

public class RunElevator extends Command {
    private ElevatorSystem elevatorSystem;
    private double speed;

    public RunElevator(ElevatorSystem elevatorSystem, double speed) {
        this.elevatorSystem = elevatorSystem;
        this.speed = speed;

        addRequirements(elevatorSystem);
    }

    @Override
    public void execute() 
    {
        elevatorSystem.run(speed);
    }
  
    @Override
    public void end(boolean interrupted) 
    {
        elevatorSystem.run(0);
    }
}
