// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  CANifier RGBLights = new CANifier(6);
  double R = 0;
  double G = 0;
  double B = 0;
  boolean enabled = false;
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
  }

  @Override
  public void periodic() {
    setColor(100, 100, 100);
    RGBLights.enablePWMOutput(0, enabled);
    RGBLights.setLEDOutput(R, LEDChannel.LEDChannelA);
    RGBLights.setLEDOutput(G, LEDChannel.LEDChannelB);
    RGBLights.setLEDOutput(B, LEDChannel.LEDChannelC);
  }

  public void setColor(double R, double G, double B) {
    this.R = R;
    this.G = G;
    this.B = B;
  }
}
