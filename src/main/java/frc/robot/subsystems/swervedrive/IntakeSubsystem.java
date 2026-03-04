package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import limelight.Limelight;

public class IntakeSubsystem extends SubsystemBase{

    SparkMax intakeMax = new SparkMax(Constants.IntakeConstants.FeederMotorID, MotorType.kBrushless);
    SparkMax intakeMoverMax = new SparkMax(Constants.IntakeConstants.MoverMotorID, MotorType.kBrushless);
   
    DigitalInput m_toplimitswitch = new DigitalInput(0);
    DigitalInput m_bottomlimitswitch = new DigitalInput(1);

    double intakeMoverSpeedConstant = 0.5;
    double intakeMoverSpeed = intakeMoverSpeedConstant;

    double intakeSpeed = 0.5;

    boolean topLimitSwitchTriggered = false;
    boolean botLimitSwitchTriggered = false;
    
    // Directions: false -> toward in (up), true -> toward out (down)
    boolean isIntakeOut;

     public IntakeSubsystem(){     
      isIntakeOut = false;
    }


  public Command Swap(){
      /**
       * Run only once when the A button is down.
       * 
       * Swaps the direction the intake mechanism is moving.
       */
      return run( () -> {
        
        var isTopPressed = m_toplimitswitch.get();
        var isBottomPressed = m_bottomlimitswitch.get();

        if (isTopPressed == true){
          System.out.println("STOP UP");
          isIntakeOut = false;
          System.out.println("isIntakeOut:" + isIntakeOut);
          intakeMoverSpeed = 0;
  
        }
        if (isBottomPressed == true){
          System.out.println("STOP DOWN");
          isIntakeOut = true;          
          System.out.println("isIntakeOut:" + isIntakeOut);
          intakeMoverSpeed = 0;
        }

        if (isIntakeOut == false){
          intakeMoverSpeed = -intakeMoverSpeedConstant;

        }
        if (isIntakeOut == true){
          intakeMoverSpeed = intakeMoverSpeedConstant;
    
        }

        if (isTopPressed || isBottomPressed){
          intakeMoverSpeed = 0;

        }
        intakeMax.set(intakeSpeed);
        intakeMoverMax.set(intakeMoverSpeed);
        System.out.println (intakeMoverSpeed);
      });
    }
}
