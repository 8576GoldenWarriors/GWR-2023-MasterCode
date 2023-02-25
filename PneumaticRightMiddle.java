// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class PneumaticRightMiddle extends CommandBase {
  /** Creates a new PneumaticRightMiddle. */
  public PneumaticRightMiddle() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Pneumatics.getPotentiometerRightAngle() > 15){
      Pneumatics.rightRunForward();
    }
    else if (Pneumatics.getPotentiometerRightAngle() < 75){
      Pneumatics.rightRunReverse();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Pneumatics.getPotentiometerRightAngle() > 40 && Pneumatics.getPotentiometerRightAngle() < 50){
      Pneumatics.rightEqualize();
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
