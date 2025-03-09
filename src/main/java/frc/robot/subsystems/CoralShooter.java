// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralShooter extends SubsystemBase {
  /** Creates a new CoralShooter. */
  public SparkMax rightMotor = new SparkMax(42, MotorType.kBrushless);
  public SparkMax leftMotor = new SparkMax(43, MotorType.kBrushless);
  public CANrange rangeSensor =  new CANrange(0);


  public CoralShooter() 
  { 
    SparkMaxConfig leftConfig = new SparkMaxConfig();
     leftConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
            //Set Current Limit
        leftConfig.smartCurrentLimit(40);
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
     rightConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
            //Set Current Limit
        rightConfig.smartCurrentLimit(40);
    rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Voltage of Coral", leftMotor.getOutputCurrent());
  }
}
