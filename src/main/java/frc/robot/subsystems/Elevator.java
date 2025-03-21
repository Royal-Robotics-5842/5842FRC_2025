// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  public SparkMax leftMotor = new SparkMax(01, MotorType.kBrushless);
  
  public SparkMax rightMotor = new SparkMax(02, MotorType.kBrushless);
  public SparkClosedLoopController RightelevPID;
  public SparkClosedLoopController LeftelevPID;
  public DigitalInput eleTopLimitSwitch;
  public DigitalInput eleBottomLimitSwitch;

  public ProfiledPIDController elevPID = new ProfiledPIDController(
    .08,
    0,
    0,
    new TrapezoidProfile.Constraints(10, 5));


  public Elevator() 
  {
     SparkMaxConfig leftConfig = new SparkMaxConfig();
     leftConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
            //Set Current Limit
        leftConfig.smartCurrentLimit(40);
        leftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder); // NEED TO CHANGE
        leftConfig.closedLoop.pid(0.1,0,0);
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig
           .inverted(true)
           .idleMode(IdleMode.kBrake);
           //Set Current Limit
  rightConfig.smartCurrentLimit(40);
  rightConfig.follow(leftMotor, true);
  rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  LeftelevPID = leftMotor.getClosedLoopController();


  eleTopLimitSwitch = new DigitalInput(1);  
  eleBottomLimitSwitch = new DigitalInput(2);

  }

  public void moveElevator(double position)
  {
    //leftMotor.set(elevPID.calculate(leftMotor.getEncoder().getPosition(), position));
    LeftelevPID.setReference(position,ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public double getPosition()
  {
    return leftMotor.getEncoder().getPosition();
  }

  public boolean getTopLimit()
  {
    return eleTopLimitSwitch.get();
  }

  public boolean getBottomLimit()
  {
    return eleBottomLimitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("EleTopLimit", eleTopLimitSwitch.get());
    SmartDashboard.putBoolean("EleBottomLimit", eleBottomLimitSwitch.get());
    SmartDashboard.putNumber("ElevatorRightEncoder", leftMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator Current", leftMotor.getOutputCurrent());
    SmartDashboard.putNumber("Elevator Power", leftMotor.getEncoder().getVelocity());

  }
}
