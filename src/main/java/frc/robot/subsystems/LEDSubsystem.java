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
  int blink_count = 0;
  int blinkPerSecond = 0;
  double powerPercentage = 100;
  boolean blinking = false;
  boolean powered = false;
  
  public enum Modes {
    IDLE,
    CORAL_RUN,
    ALAGE_RUN,
    TELEOP,
    AUTO
  }
  Modes currentMode = Modes.IDLE;
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    setPowerPercentage(50);
    enable();
  }

  public void enable() {
    powered = true;
  }

  public void disable() {
    powered = false;
  }

  public void setMode(Modes mode) {
    currentMode = mode;

    switch (mode) {
      case IDLE:
        //Royal Purple
        setColor(47,32,66);
        setBlinking(false, 0);
        break;
      case CORAL_RUN:
        //Royal Blue
        setColor(65,105,225);
        setBlinking(false, 0);
        break;
      case ALAGE_RUN:
        //Royal Green
        setColor(28,101,27);
        setBlinking(false, 0);
        break;
      case TELEOP:
        //Royal Purple
        setColor(47,32,66);
        setBlinking(true, 5);
        break;
      case AUTO:
        //Royal Orange
        setColor(249,146,69);
        setBlinking(true, 5);
        break;
  
    }
  }

  public void setPowerPercentage(double percentage) {
    this.powerPercentage = percentage;
  }

  public void setBlinking(boolean blinking, int timesPerSecond) {
    this.blinking = blinking;
    this.blinkPerSecond = timesPerSecond;
  }

  public void setColor(int R, int G, int B) {
    this.R = R;
    this.G = G;
    this.B = B;
  }

  @Override
  public void periodic() {
    double duty_cycle =  (0.001 / 0.01) * powerPercentage; 
    RGBLights.enablePWMOutput(0, powered);
    RGBLights.setLEDOutput(G, LEDChannel.LEDChannelA);
    RGBLights.setLEDOutput(R, LEDChannel.LEDChannelB);
    RGBLights.setLEDOutput(B, LEDChannel.LEDChannelC);
    RGBLights.setPWMOutput(0, 1000 / duty_cycle);

    if(blinking && blink_count >= blinkPerSecond) {
      blink_count = 0;
    }

    blink_count++;

    setColor(100, 100, 100);
    RGBLights.enablePWMOutput(0, powered);
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
