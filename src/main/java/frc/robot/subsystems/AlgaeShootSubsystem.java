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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeShootSubsystem extends SubsystemBase {
      /** Creates a new ArmSubsystem. */
    public SparkMax rightMotor = new SparkMax(62, MotorType.kBrushless);
    public SparkMax leftMotor = new SparkMax(61, MotorType.kBrushless);
    public SparkClosedLoopController RightarmPID;
    public SparkClosedLoopController LeftarmPID;

    public AlgaeShootSubsystem() {
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);
                //Set Current Limit
        leftConfig.smartCurrentLimit(40);
        leftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        leftConfig.closedLoop.pid(0.05,0,0);
            leftConfig.smartCurrentLimit(40);
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig
                .inverted(true)
                .idleMode(IdleMode.kBrake);
                //Set Current Limit
            rightConfig.smartCurrentLimit(40);
            rightConfig.follow(leftMotor, true);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        LeftarmPID = leftMotor.getClosedLoopController();
    }

      public void moveArm(double position)
        {
            LeftarmPID.setReference(position,ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}
