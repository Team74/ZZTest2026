package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import limelight.Limelight;

public class IntakeSubsystem extends SubsystemBase{

    SparkMax intakeMax = new SparkMax(Constants.IntakeConstants.FeederMotorID, MotorType.kBrushless);
    SparkMax intakeMoverMax = new SparkMax(Constants.IntakeConstants.MoverMotorID, MotorType.kBrushless);
   
    PIDController intakePID;
    boolean isIntakeOut;
    double target;

    double intakeSpeed = 0.5;
    double reduceMoverSpeed = 0.25;

    double upEncoderValue = 0;
    double downEncoderValue = 10;
     public IntakeSubsystem(){     
      SparkMaxConfig moverConfig = new SparkMaxConfig();
      moverConfig.smartCurrentLimit(105);
      moverConfig.idleMode(IdleMode.kBrake);

      intakeMoverMax.configure(moverConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      intakeMoverMax.getEncoder().setPosition(0);

      intakePID = new PIDController(.05, 0, 0);
      //ProfiledPIDController intakePID = new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(120, 180));
    
      intakePID.setTolerance(0.1);
      intakePID.disableContinuousInput();

      isIntakeOut = false;
    }

  
    public Command intakeIn(){
      return run(()->{
        intakeMax.set(-intakeSpeed);
        System.out.println(intakeSpeed);
      });
    } 

    public Command intakeOut(){
      return run(()->{
        intakeMax.set(-intakeSpeed);
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
        System.out.println("isIntakeOut: " + isIntakeOut + " target: + " + target + "Current Pos: " + intakeMoverMax.getEncoder().getPosition() + " Current: " + intakeMoverMax.getOutputCurrent());
        intakeMoverMax.set(pidTarget*reduceMoverSpeed);
      });
    }

    public Command Swap(){
      return run(()->{
        var currentPos = intakeMoverMax.getEncoder().getPosition();

        if(currentPos >= downEncoderValue) {
            isIntakeOut = true;
        }
        else if(currentPos <= upEncoderValue) {
            isIntakeOut = false;
        }
        System.out.println("isIntakeOut: " + isIntakeOut + " target: + " + target + "Current Pos: " + intakeMoverMax.getEncoder().getPosition() + " Current: " + intakeMoverMax.getOutputCurrent());

        if (isIntakeOut){
            // moving the target from 11.4 to zero
            // 0 = in, 11.4 = out
            target = upEncoderValue;
        }
        if (!isIntakeOut) {
            target = downEncoderValue;
        }
      });
    }
}
