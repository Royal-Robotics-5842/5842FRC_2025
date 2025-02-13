// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ShootCoral;
import frc.robot.commands.SwerveDriveJoystick;
import frc.robot.commands.SwerveDriveJoystickLimeLight;
import frc.robot.commands.elevPID;
import frc.robot.commands.moveElevator;
import frc.robot.commands.resetEverything;
import frc.robot.subsystems.CoralShooter;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;
=======
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.RunElevator;
import frc.robot.commands.RunElevatorPID;
import frc.robot.commands.ShootCoral;
import frc.robot.commands.SwerveDriveJoystick;
import frc.robot.commands.SwerveDriveJoystickLimeLight;
import frc.robot.commands.elevPID;
import frc.robot.commands.moveElevator;
import frc.robot.commands.resetEverything;
import frc.robot.subsystems.CoralShooter;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
>>>>>>> b1ec55dfe6960c20357ccbc0d076a323fad86dc6

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
<<<<<<< HEAD
  private final Elevator elevator = new Elevator();
=======
  private final ElevatorSystem elevator = new ElevatorSystem();
>>>>>>> b1ec55dfe6960c20357ccbc0d076a323fad86dc6
  private final CoralShooter coral = new CoralShooter();


  //THe robot's commands are defined here...

  //All our controller stuff!
  public final static Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
  public final static CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  public final static double lefTrigger = m_driverController.getLeftTriggerAxis();
  public final static double rightTrigger = m_driverController.getRightTriggerAxis();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   
    swerveSubsystem.setDefaultCommand(new SwerveDriveJoystick(
      swerveSubsystem,
      () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis), // Forward/Back DO NOT TOUCH
      () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis), // Left/Right
      () -> -driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
      () -> !m_driverController.y().getAsBoolean()));
    
      SmartDashboard.putBoolean("Field Centric", !m_driverController.y().getAsBoolean());
      SmartDashboard.putNumber("Robot Pitch", swerveSubsystem.gyro.getPitch());

    configureBindings();
<<<<<<< HEAD
    
=======
    
    SmartDashboard.putNumber("Turn BL", swerveSubsystem.backLeft.getTurningPosition());
    SmartDashboard.putNumber("Turn FL", swerveSubsystem.frontLeft.getTurningPosition());
    SmartDashboard.putNumber("Turn BR", swerveSubsystem.backRight.getTurningPosition());
    SmartDashboard.putNumber("Turn FR", swerveSubsystem.frontRight.getDrivePosition());
  
    //Register auto commands
    NamedCommands.registerCommand("Coral Outtake", new ShootCoral(coral, 0.10));
    NamedCommands.registerCommand("L4 Elevator", new RunElevatorPID(elevator, 183));
    NamedCommands.registerCommand("L3 Elevator", new RunElevatorPID(elevator, 103));
    NamedCommands.registerCommand("L2 Elevator", new RunElevatorPID(elevator, 47));
    NamedCommands.registerCommand("L1/Bottom Elevator", new RunElevatorPID(elevator, 3));
>>>>>>> b1ec55dfe6960c20357ccbc0d076a323fad86dc6
    
    
  }

  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.rightTrigger().onTrue((new resetEverything(swerveSubsystem,elevator)).withTimeout(0.5));
    
    m_driverController.a().whileTrue(new SwerveDriveJoystickLimeLight(
      swerveSubsystem,
      () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis), // Forward/Back DO NOT TOUCH
      () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis), // Left/Right
      () -> !m_driverController.y().getAsBoolean()));

    m_driverController.y().toggleOnTrue(new moveElevator(elevator, 0.05));
    m_driverController.x().toggleOnTrue(new moveElevator(elevator, -0.75));
    m_driverController.b().toggleOnTrue(new moveElevator(elevator, 0));

<<<<<<< HEAD
=======

>>>>>>> b1ec55dfe6960c20357ccbc0d076a323fad86dc6
    m_driverController.povUp().toggleOnTrue(new elevPID(elevator, 178));
    m_driverController.povLeft().toggleOnTrue(new elevPID(elevator, 103));
    m_driverController.povRight().toggleOnTrue(new elevPID(elevator, 47));
    m_driverController.povDown().toggleOnTrue(new elevPID(elevator, 1));
    

<<<<<<< HEAD
    m_driverController.leftBumper().toggleOnTrue(new ShootCoral(coral, 0.10));
=======
    m_driverController.leftBumper().toggleOnTrue(new ShootCoral(coral, 0.1));
>>>>>>> b1ec55dfe6960c20357ccbc0d076a323fad86dc6
  }
    
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
        // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the scommand to run in autonomous
   */
  
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Elevator Auto");
  }
}

