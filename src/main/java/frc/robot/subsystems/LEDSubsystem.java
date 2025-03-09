// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  CANifier RGBLights = new CANifier(6);
  
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
  }

  @Override
  public void periodic() {
    RGBLights.enablePWMOutput(0, true);
    RGBLights.setLEDOutput(50, LEDChannel.LEDChannelA);
    RGBLights.setLEDOutput(0, LEDChannel.LEDChannelB);
    RGBLights.setLEDOutput(50, LEDChannel.LEDChannelC);
  }
}
