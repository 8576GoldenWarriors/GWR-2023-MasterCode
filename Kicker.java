// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Kicker extends SubsystemBase {
  public static VictorSPX kicker = new VictorSPX(5);
  /** Creates a new Kicker. */
  public Kicker() {
    kicker.configFactoryDefault();
  }

  public static void kicker(double speed) {
    kicker.set(VictorSPXControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
