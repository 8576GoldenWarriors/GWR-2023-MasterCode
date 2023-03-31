// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class Cone_Intake extends CommandBase {
  /** Creates a new Shoot. */
  public double speed;
  public Cone_Intake(Intake Intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Intake.intake(-0.3);
    Kicker.kicker(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake.intake(0);
    Kicker.kicker(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
