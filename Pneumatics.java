package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
    
    //uncomment this for pneumatics
    //Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.REVPH);
    static DoubleSolenoid doubleSolenoidLeft1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
    static DoubleSolenoid doubleSolenoidleft2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);

    static DoubleSolenoid doubleSolenoidRight1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);
    static DoubleSolenoid doubleSolenoidRight2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8, 9);

    static AnalogPotentiometer potentiometerLeft = new AnalogPotentiometer(0, 90, 0);
    static AnalogPotentiometer potentiometerRight = new AnalogPotentiometer(1, 90, 0);


    public Pneumatics(){
        doubleSolenoidLeft1.set(Value.kOff);
        doubleSolenoidleft2.set(Value.kOff);
        doubleSolenoidRight1.set(Value.kOff);
        doubleSolenoidRight2.set(Value.kOff);
    }

    public static void runForward(){
        doubleSolenoidLeft1.set(Value.kForward);
        doubleSolenoidleft2.set(Value.kReverse);
        doubleSolenoidRight1.set(Value.kForward);
        doubleSolenoidRight2.set(Value.kReverse);
    }

    public static void leftRunForward(){
        doubleSolenoidLeft1.set(Value.kForward);
        doubleSolenoidleft2.set(Value.kReverse);
    }

    public static void rightRunForward(){
        doubleSolenoidRight1.set(Value.kForward);
        doubleSolenoidRight2.set(Value.kReverse);
    }

    public static void runReverse(){
        doubleSolenoidLeft1.set(Value.kReverse);
        doubleSolenoidleft2.set(Value.kForward);
        doubleSolenoidRight1.set(Value.kReverse);
        doubleSolenoidRight2.set(Value.kForward);
    }

    public static void leftRunReverse(){
        doubleSolenoidLeft1.set(Value.kReverse);
        doubleSolenoidleft2.set(Value.kForward);
    }

    public static void rightRunReverse(){
        doubleSolenoidRight1.set(Value.kReverse);
        doubleSolenoidRight2.set(Value.kForward);
    }

    

    // public void middle(){
    //     double angle = potentiometerLeft.get() * 90;

    //     if (angle < 39 && angle > 0){

    //         //System.out.println(1.0);
    //         doubleSolenoidLeft1.set(Value.kForward);
    //         doubleSolenoidleft2.set(Value.kReverse);

    //     }

    //     if (angle > 51 && angle < 90){

    //         //System.out.println(2.0);
    //         doubleSolenoidLeft1.set(Value.kReverse);
    //         doubleSolenoidleft2.set(Value.kForward);

    //     }

    //     if (angle > 40 && angle < 50){

    //         //System.out.println(3.0);
    //         doubleSolenoidLeft1.set(Value.kOff);

    //     }
    // }

    public static void leftEqualize(){
        doubleSolenoidLeft1.set(Value.kForward);
        doubleSolenoidleft2.set(Value.kForward);
    }

    public static void rightEqualize(){
        doubleSolenoidRight1.set(Value.kForward);
        doubleSolenoidRight2.set(Value.kForward);
    }

    public static double getPotentiometerLeftAngle(){
        return potentiometerLeft.get();
    }

    public static double getPotentiometerRightAngle(){
        return potentiometerRight.get();
    }

}