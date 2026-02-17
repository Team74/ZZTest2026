package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import limelight.Limelight;

public class IntakeSubsystem extends SubsystemBase{

    SparkMax intakeMax;
    SparkMax hoodMotor;
   
    double speed;
    
    PIDController intakePID;
    SparkMax intakeMoverMax;
    SparkMaxConfig thing;
    boolean isIntakeOut;
    double target;
    double lim;
    XboxController driveController = new XboxController(0);
    XboxController operatorController = new XboxController(1);
    Limelight limelight3 = new Limelight("limelight3");

    public IntakeSubsystem(){
        intakeMax = new SparkMax(12, MotorType.kBrushless);
        hoodMotor = new SparkMax(3, MotorType.kBrushless); 
        
        intakePID = new PIDController(.05, 0, 0);
        intakePID.setTolerance(0.1);
        intakePID.disableContinuousInput();

        intakeMoverMax = new SparkMax(33, MotorType.kBrushless);
        intakeMoverMax.getEncoder().setPosition(81);
        isIntakeOut = false;
        thing = new SparkMaxConfig();

    thing.smartCurrentLimit(105);
    intakeMoverMax.configure(thing, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        lim = intakeMoverMax.configAccessor.getSmartCurrentLimit();
        //driveController = new XboxController(0);

    }
    public Command intakeIn(){
        return run(()->{
            intakeMax.set(0.5);
        });
    } 

    public Command intakeOut(){
        return run(()->{
            intakeMax.set(-0.5);
        });
    } 

        public Command intakeStop(){
        return run(()->{
            intakeMax.set(0);
            
        });
    } 

    public Command Moveintake(){
        return run(()-> {
            var pidTarget = intakePID.calculate(intakeMoverMax.getEncoder().getPosition(), target);
           
            System.out.println("pidTarget: " + pidTarget);

            intakeMoverMax.set(pidTarget);
    });}

    /*public Command hoodLimeTarget(){

         float KpDistance = -0.1f;  // Proportional control constant for distance
        float current_distance = Estimate_Distance();  // see the 'Case Study: Estimating Distance' 

         return run(()->{
            hoodMotor.set(0.25);
        });

    }*/

    public Command MoveHoodOut(){

        return run(()->{
            hoodMotor.set(0.25);
         
        });

    }

    public Command MoveHoodIn(){

        return run(()->{
            hoodMotor.set(-0.25);
           
        });

    }

    public Command StopHood(){

        return run(()->{
            hoodMotor.set(0);
            
        });

    }

    public Command Swap(){
        return run(()->{
                var currentPos = intakeMoverMax.getEncoder().getPosition();

                if(currentPos <= 78.73) {
                    isIntakeOut = true;
                }
                else if(currentPos >= 80.9) {
                    isIntakeOut = false;
                }



                        System.out.println("isIntakeOut: " + isIntakeOut + " target: + " + target + "Current Pos: " + intakeMoverMax.getEncoder().getPosition() + " Current: " + intakeMoverMax.getOutputCurrent() +" Current Lim: " + lim);

            // if (intakePID.atSetpoint() && target > 78.73) {
            //     isIntakeOut = true;
            // } else if (intakePID.atSetpoint() && target <= 81) {
            //     isIntakeOut = false;
            // }

            if (isIntakeOut){
                target = 81;
            }
            if (!isIntakeOut) {
                target = 78.73;
            }


        });
    }
            
        
    
}
