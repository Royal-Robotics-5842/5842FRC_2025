// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmMoveSubsystem extends SubsystemBase {
  /** Creates a new Elevator. */
  public SparkMax motor = new SparkMax(52, MotorType.kBrushless);
  public SparkClosedLoopController MotorPID;
  public ProfiledPIDController armPidController = new ProfiledPIDController(
    .06,
    0,
    0,
    new TrapezoidProfile.Constraints(30, 30));
  /*
  * Creates a new ArmMoveSubsystem. */

  public ArmMoveSubsystem() {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
     leftConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
            //Set Current Limit
        leftConfig.smartCurrentLimit(40);
    motor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    MotorPID = motor.getClosedLoopController();
  }
  public void moveArm(double position)
  {
    //MotorPID.setReference(position,ControlType.kPosition, ClosedLoopSlot.kSlot0);
    /*
    if (motor.getAppliedOutput() < 0)
    {
      motor.set(armPidController.calculate(motor.getEncoder().getPosition(), position)/2);
    }
      */
    motor.set(armPidController.calculate(motor.getEncoder().getPosition(), position));

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ARM ENCODER", motor.getEncoder().getPosition());
    SmartDashboard.putNumber("Arm Power", motor.getAppliedOutput());
  }


}
