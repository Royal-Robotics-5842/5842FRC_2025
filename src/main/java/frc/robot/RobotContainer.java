// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashSet;
import java.util.Set;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ShootAlgae;
import frc.robot.commands.ShootCoral;
import frc.robot.commands.SwerveDriveJoystick;
import frc.robot.commands.armPID;
import frc.robot.commands.elevPID;
import frc.robot.commands.moveArm;
import frc.robot.commands.moveElevator;
import frc.robot.commands.onTheFlyPathPlanner;
import frc.robot.commands.resetEverything;
import frc.robot.subsystems.AlgaeShootSubsystem;
import frc.robot.subsystems.ArmMoveSubsystem;
import frc.robot.subsystems.CoralShooter;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Elevator elevator = new Elevator();
  private final CoralShooter coral = new CoralShooter();
  private final ArmMoveSubsystem arm = new ArmMoveSubsystem();
  private final AlgaeShootSubsystem algae = new AlgaeShootSubsystem();


  //THe robot's commands are defined here...

  //All our controller stuff!
  public final static Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
  public final static CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  public final static double lefTrigger = m_driverController.getLeftTriggerAxis();
  public final static double rightTrigger = m_driverController.getRightTriggerAxis();

  public final static CommandXboxController operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);
  public final static double o_lefTrigger = operatorController.getLeftTriggerAxis();
  public final static double o_rightTrigger = operatorController.getRightTriggerAxis();

  Set< Subsystem > set = new HashSet<>(Set.of( swerveSubsystem ));
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
/*
    m_driverController.a().whileTrue(new SwerveDriveJoystickLimeLight(
      swerveSubsystem,

      () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis), // Forward/Back DO NOT TOUCH
      () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis), // Left/Right
      () -> !m_driverController.y().getAsBoolean()));
*/
    m_driverController.y().toggleOnTrue(new moveElevator(elevator, 0.05));
    m_driverController.x().toggleOnTrue(new moveElevator(elevator, -0.25));
    m_driverController.b().toggleOnTrue(new moveElevator(elevator, 0.25));

    m_driverController.povUp().toggleOnTrue(new elevPID(elevator, 17));
    m_driverController.povLeft().toggleOnTrue(new elevPID(elevator, 94));
    m_driverController.povRight().toggleOnTrue(new elevPID(elevator, 40));
    m_driverController.povDown().toggleOnTrue(new elevPID(elevator, 1));

    m_driverController.leftBumper().whileTrue(new ShootAlgae( algae, 1.5));
    m_driverController.rightBumper().whileTrue(new ShootAlgae( algae, -1));

    m_driverController.rightTrigger().whileTrue(new moveArm(arm, -0.25));
    m_driverController.leftTrigger().whileTrue(new moveArm(arm, 0.25));
    
   
    //new onTheFlyPathPlanner(swerveSubsystem, 
    //new Pose2d(3.180 , 4.193, Rotation2d.fromDegrees(0))));
    /*/  
    swerveSubsystem.gotoPath(
    PathPlannerPath.waypointsFromPoses( 
    new Pose2d(3 , 4, Rotation2d.fromDegrees(0)),
    new Pose2d(3.180 , 4.193, Rotation2d.fromDegrees(0)))));
*/




m_driverController.a().whileTrue(Commands.defer(() -> {
  return swerveSubsystem.gotoPath(PathPlannerPath.waypointsFromPoses(
    swerveSubsystem.SwerveDrivePoseEstimator.getEstimatedPosition(),
    new Pose2d(3.8, 4.193, Rotation2d.fromDegrees(0)))); // Return an empty command after execution
  }, set));

    
  
    operatorController.leftBumper().whileTrue(new ShootCoral(coral, -0.25));
    operatorController.rightBumper().whileTrue(new ShootCoral(coral, -0.1));
    operatorController.rightTrigger().onTrue((new resetEverything(swerveSubsystem,elevator)).withTimeout(0.5));
    operatorController.a().onTrue(new armPID(arm, 33));
    operatorController.b().onTrue(new armPID(arm,2));

    NamedCommands.registerCommand("Coral Outtake", new ShootCoral(coral, 0.10));
    NamedCommands.registerCommand("L4 Elevator", new elevPID(elevator, 183));
    NamedCommands.registerCommand("L3 Elevator", new elevPID(elevator, 87));
    NamedCommands.registerCommand("L2 Elevator", new elevPID(elevator, 34));
    NamedCommands.registerCommand("L1/Bottom Elevator", new elevPID(elevator, 3));
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
    return new PathPlannerAuto("IKA Reef Cycle");
  }
}