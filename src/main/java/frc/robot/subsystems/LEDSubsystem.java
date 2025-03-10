// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  static CANifier RGBLights = new CANifier(6);
  static double R = 0;
  static double G = 0;
  static double B = 0;
  static int blink_count = 0;
  static int blinkDelay = 0;
  static double powerDividend = 100;
  static boolean blinking = false;
  static boolean powered = false;

  public static enum Modes {
    IDLE,
    CORAL_RUN,
    ALAGE_RUN,
    TELEOP,
    AUTO
  }

  static Modes currentMode = Modes.IDLE;
  public static Modes robotMode = Modes.IDLE;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
  }

  public static void enable() {
    powered = true;
  }

  public void disable() {
    powered = false;
  }

  public static void setMode(Modes mode) {
    currentMode = mode;

    switch (mode) {
      case IDLE:
        // Royal Purple
        setColor(1, 0, 1);
        setBlinking(false, 0);
        break;
      case CORAL_RUN:
        // Blue
        setColor(0, 0, 1);
        setBlinking(true, 8);
        break;
      case ALAGE_RUN:
        // Green
        setColor(0, 1, 0);
        setBlinking(true, 8);
        break;
      case TELEOP:
        // Purple
        setColor(1, 0, 1);
        setBlinking(true, 20);
        break;
      case AUTO:
        // Orange
        setColor(1, .1, 0);
        setBlinking(true, 20);
        break;
    }
  }

  public static void cutPowerBy(double dividend) {
    LEDSubsystem.powerDividend
 = dividend;
  }

  public static void setBlinking(boolean newBlink, int blinkingDelay) {
    blinking = newBlink;
    blinkDelay = blinkingDelay;

    if (blinking == false) {
      enable();
    }
  }

  public static void setColor(double R2, double G2, double B2) {
    R = R2;
    G = G2;
    B = B2;
  }

  @Override
  public void periodic() {
    // double duty_cycle = (0.001 / 0.01) * powerDividend
;
    if (powered) {
      RGBLights.setLEDOutput(G / powerDividend, LEDChannel.LEDChannelA);
      RGBLights.setLEDOutput(R / powerDividend, LEDChannel.LEDChannelB);
      RGBLights.setLEDOutput(B / powerDividend, LEDChannel.LEDChannelC);
    } else {
      RGBLights.setLEDOutput(0, LEDChannel.LEDChannelA);
      RGBLights.setLEDOutput(0, LEDChannel.LEDChannelB);
      RGBLights.setLEDOutput(0, LEDChannel.LEDChannelC);
    }

    if (blinking && blink_count >= blinkDelay) {
      blink_count = 0;
      powered = !powered;
    }

    blink_count++;
  }
}
