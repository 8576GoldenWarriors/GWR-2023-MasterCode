// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class PneumaticLeftMiddle extends CommandBase {
  /** Creates a new PneumaticLeftMiddle. */

  public PneumaticLeftMiddle(Pneumatics pneumatics) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pneumatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Pneumatics.getPotentiometerLeftAngle() > 15 && Pneumatics.getPotentiometerLeftAngle() < 49){
      Pneumatics.leftRunForward();
    }
    else if (Pneumatics.getPotentiometerLeftAngle() < 75 && Pneumatics.getPotentiometerLeftAngle() > 52){
      Pneumatics.leftRunReverse();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(Pneumatics.getPotentiometerLeftAngle());
    if (Pneumatics.getPotentiometerLeftAngle() > 38 && Pneumatics.getPotentiometerLeftAngle() < 43){
      Pneumatics.leftEqualize();
    }
    else if (Pneumatics.getPotentiometerRightAngle() > 15 && Pneumatics.getPotentiometerRightAngle() < 37){
      Pneumatics.rightRunForward();
    }
    else if (Pneumatics.getPotentiometerRightAngle() < 75 && Pneumatics.getPotentiometerRightAngle() > 44){
      Pneumatics.rightRunReverse();
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
