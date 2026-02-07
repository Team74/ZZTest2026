package frc.robot.subsystems.swervedrive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{

    SparkMax intakeMax;
   
    double speed;
    
    PIDController intakePID;
    SparkMax intakeMoverMax;
    
    boolean isIntakeOut;
    double target;

    XboxController driveController = new XboxController(0);

    public IntakeSubsystem(){
        intakeMax = new SparkMax(12, MotorType.kBrushless);
        
        intakePID = new PIDController(.5, 0, 0);
        intakeMoverMax = new SparkMax(33, MotorType.kBrushless);
        
        isIntakeOut = false;
        
        driveController = new XboxController(0);

    }
    public Command intakeIn(){
        return run(()->{
            intakeMax.set(.20);
        });
    } 

    public Command intakeOut(){
        return run(()->{
            intakeMax.set(-.20);
            
        });
    } 

    public Command Moveintake(){
        return run(()-> {

            intakeMoverMax.set(intakePID.calculate(intakeMoverMax.getAbsoluteEncoder().getPosition(),target)*.2);
            });}

    public Command Swap(){
        return run(()->{

            if (intakePID.atSetpoint() && target == 90) {
                isIntakeOut = true;
            } else if (intakePID.atSetpoint() && target == 0) {
                isIntakeOut = false;
            }
            
            if (isIntakeOut){
                target = 0;
            }
            if (!isIntakeOut) {
                target = 90;
            }


        });
    }
            
        
    
}
