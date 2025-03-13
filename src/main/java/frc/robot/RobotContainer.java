// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashSet;
import java.util.Set;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.AutoConstants.Side;
import frc.robot.commands.Positoning;
import frc.robot.commands.ShootAlgae;
import frc.robot.commands.ChangeLED;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.OuttakeCoral;
import frc.robot.commands.SwerveDriveJoystick;
import frc.robot.commands.armPID;
import frc.robot.commands.elevPID;
import frc.robot.commands.moveArm;
import frc.robot.commands.moveElevator;
import frc.robot.commands.resetEverything;
import frc.robot.subsystems.AlgaeShootSubsystem;
import frc.robot.subsystems.ArmMoveSubsystem;
import frc.robot.subsystems.CoralShooter;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LEDSubsystem.Modes;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  // The robot's subsystems are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Elevator elevator = new Elevator();
  private final CoralShooter coral = new CoralShooter();
  private final ArmMoveSubsystem arm = new ArmMoveSubsystem();
  private final AlgaeShootSubsystem algae = new AlgaeShootSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();


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
    configureBindings();

    swerveSubsystem.setDefaultCommand(new SwerveDriveJoystick(
      swerveSubsystem,
      () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis), // Forward/Back DO NOT TOUCH
      () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis), // Left/Right
      () -> -driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
      () -> !m_driverController.y().getAsBoolean(),
      elevator));
    
      SmartDashboard.putBoolean("Field Centric", !m_driverController.y().getAsBoolean());
      
    LEDSubsystem.cutPowerBy(20);
    LEDSubsystem.enable();

    NamedCommands.registerCommand("OutCoral", new OuttakeCoral(coral, -0.5));
    NamedCommands.registerCommand("IntakeCoral", new IntakeCoral(coral, -0.1));
    NamedCommands.registerCommand("L4 Elevator", new elevPID(elevator, Constants.elevatorConstants.L4_height));
    NamedCommands.registerCommand("L3 Elevator", new elevPID(elevator, Constants.elevatorConstants.L3_height));
    NamedCommands.registerCommand("L2 Elevator", new elevPID(elevator, Constants.elevatorConstants.L2_height));
    NamedCommands.registerCommand("BottomElevator", new elevPID(elevator, Constants.elevatorConstants.bottom_height));
  
    autoChooser = AutoBuilder.buildAutoChooser("Testing");
    SmartDashboard.putData("Auto Chooser", autoChooser);
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
      
    m_driverController.leftTrigger().whileTrue(Commands.defer(() -> {
      return new Positoning(swerveSubsystem,Side.left); // Return an empty command after execution
      }, set));
    m_driverController.rightTrigger().whileTrue(Commands.defer(() -> {
      return new Positoning(swerveSubsystem, Side.right); // Return an empty command after execution
      }, set));


    m_driverController.y().toggleOnTrue(new ParallelCommandGroup(
      new armPID(arm, Constants.armConstants.stow),      
      new elevPID(elevator, Constants.elevatorConstants.L4_height)
    ));
    m_driverController.x().toggleOnTrue(
      new ParallelCommandGroup(
      new armPID(arm, Constants.armConstants.stow),      
      new elevPID(elevator, Constants.elevatorConstants.L3_height)
    ));
    m_driverController.b().toggleOnTrue(new elevPID(elevator, Constants.elevatorConstants.L2_height));
    m_driverController.a().toggleOnTrue(new ParallelCommandGroup(
      new armPID(arm, Constants.armConstants.stow),      
      new elevPID(elevator, Constants.elevatorConstants.bottom_height)));

    m_driverController.leftBumper().toggleOnTrue(new IntakeCoral(coral, -0.1));//.andThen(new OuttakeCoral(coral, 0.25).withTimeout(0.1)));
    m_driverController.rightBumper().toggleOnTrue(new OuttakeCoral(coral, -0.5));
 

    //------------OPERATOR------------
    operatorController.leftTrigger().whileTrue(new ShootAlgae(algae, 1));
    operatorController.rightTrigger().whileTrue(new ShootAlgae(algae, -1));
    
    operatorController.leftStick().onTrue((new resetEverything(swerveSubsystem)).withTimeout(0.1));
    //operatorController.rightTrigger().onTrue((new resetEverything(swerveSubsystem)).withTimeout(0.1));
    operatorController.a().onTrue(new armPID(arm, Constants.armConstants.groundPickup));
    operatorController.y().onTrue(new armPID(arm, Constants.armConstants.stow));

    operatorController.x().whileTrue(new moveElevator(elevator, 0.1));
    operatorController.b().whileTrue(new moveElevator(elevator, -0.1));

    operatorController.leftBumper().toggleOnTrue(
      new ParallelCommandGroup(
      new armPID(arm, Constants.armConstants.barge),      
      new elevPID(elevator, Constants.elevatorConstants.barge)
    ));
    operatorController.povUp().toggleOnTrue(
      new ParallelCommandGroup(
      new armPID(arm, Constants.armConstants.topReef),      
      new elevPID(elevator, Constants.elevatorConstants.topReef)
    ));
    operatorController.povDown().toggleOnTrue(
      new ParallelCommandGroup(
      new armPID(arm, Constants.armConstants.bottomReef),      
      new elevPID(elevator, Constants.elevatorConstants.bottomReef)
    ));
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
    return autoChooser.getSelected();
  }
}