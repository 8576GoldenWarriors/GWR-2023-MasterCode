package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Stabilize extends CommandBase{
    //public double correctAngle = DrivetrainSubsystem.navX.getAngle();
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    public Stabilize(){
        addRequirements();
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        DrivetrainSubsystem.navX.reset();
        SmartDashboard.putNumber("Gyroscope Initalize", 0);
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {


        double yRotation = DrivetrainSubsystem.navX.getYaw();
        //System.out.println("Stabilize " + yRotation);

        // System.out.println("Pitch: " + yRotation);
        System.out.println("Yaw: " + yRotation);
        // System.out.println("Roll: " + DrivetrainSubsystem.navX.getRoll());


        if (yRotation < 15 && yRotation > 2){

            DrivetrainSubsystem.arcadeDrive(0.45, 0.0);
            System.out.println("1");

        }

        else if (yRotation > -15 && yRotation < -2){

            DrivetrainSubsystem.arcadeDrive(-0.45, 0.0);
            System.out.println("2");

        }

        else{

            DrivetrainSubsystem.arcadeDrive(0.0, 0.0);
            System.out.println("3");

        }

      }
      
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) { //This does nothing as I don't know how to get end to be called - John
        DrivetrainSubsystem.arcadeDrive(0.0, 0.0);
      }
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() { //Tried messing with this so the command will end but I didn't have any luck - John

        return false;
      }

}
