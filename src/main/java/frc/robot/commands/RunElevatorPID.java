package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSystem;

public class RunElevatorPID extends Command {
    private ElevatorSystem elevatorSystem;
    private double setpoint;

    public RunElevatorPID(ElevatorSystem elevatorSystem, double setpoint) {
        this.elevatorSystem = elevatorSystem;
        this.setpoint = setpoint;

        addRequirements(elevatorSystem);
    }

    @Override
    public void execute() 
    {
        elevatorSystem.pidRun(setpoint);
    }
  
    @Override
    public void end(boolean interrupted) 
    {
        //elevatorSystem.run(0);
    }
}
