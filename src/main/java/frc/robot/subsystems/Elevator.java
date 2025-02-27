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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  public SparkMax leftMotor = new SparkMax(32, MotorType.kBrushless);
  
  public SparkMax rightMotor = new SparkMax(33, MotorType.kBrushless);
  public SparkClosedLoopController RightelevPID;
  public SparkClosedLoopController LeftelevPID;
  public DigitalInput eleLimitSwitch;

  public Elevator() 
  {
     SparkMaxConfig leftConfig = new SparkMaxConfig();
     leftConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
            //Set Current Limit
        leftConfig.smartCurrentLimit(40);
        leftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        leftConfig.closedLoop.pid(0.085,0,0);
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

  eleLimitSwitch = new DigitalInput(1);  

  }

  public void moveElevator(double position)
  {
    LeftelevPID.setReference(position,ControlType.kPosition, ClosedLoopSlot.kSlot0);
  
  }

  public boolean getLimit()
  {
    return eleLimitSwitch.get();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("EleLimit",eleLimitSwitch.get());
    SmartDashboard.putNumber("ElevatorRightEncoder", leftMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("elevator Power", leftMotor.getAppliedOutput());

  }
}
