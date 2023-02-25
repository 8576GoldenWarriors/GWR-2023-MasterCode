// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class PneumaticLeftMiddle extends CommandBase {
  /** Creates a new PneumaticLeftMiddle. */
  public PneumaticLeftMiddle() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Pneumatics.getPotentiometerLeftAngle() > 15){
      Pneumatics.leftRunForward();
    }
    else if (Pneumatics.getPotentiometerLeftAngle() < 75){
      Pneumatics.leftRunReverse();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Pneumatics.getPotentiometerLeftAngle() > 40 && Pneumatics.getPotentiometerLeftAngle() < 50){
      Pneumatics.leftEqualize();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
