package frc.robot.subsystems.swervedrive;

import java.lang.invoke.VarHandle;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
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
  SparkMax intakeMax;
  SparkMax intakeMoverMax;
  SparkMax HotdogMotor;

  DigitalInput m_toplimitswitch = new DigitalInput(4);
  DigitalInput m_bottomlimitswitch = new DigitalInput(5);

  double intakeMoverSpeedConstant = Constants.IntakeConstants.IntakeMoverSpeed;
  double intakeMoverSpeed;

  double intakeSpeed = 0.7;

  boolean topLimitSwitchTriggered = false;
  boolean botLimitSwitchTriggered = false;

  // Directions: false -> toward in (up), true -> toward out (down)
  boolean isIntakeOut;
  boolean isTopPressed;
  boolean isBottomPressed;
  boolean isMovingOut;

  public IntakeSubsystem(){     
    intakeMax = new SparkMax(Constants.IntakeConstants.FeederMotorID, MotorType.kBrushed);
    //intakeMoverMax = new SparkMax(Constants.IntakeConstants.MoverMotorID, MotorType.kBrushless);
    HotdogMotor = new SparkMax(Constants.IntakeConstants.HotdogMotorID,MotorType.kBrushless);
    isIntakeOut = false;
    isMovingOut = false;
  }

  public Command moveIntake(boolean reverse){
    return run(()->{
      double desiredSpeed = intakeSpeed;
      if(reverse){
          desiredSpeed = -desiredSpeed;
      }
      intakeMax.set(desiredSpeed);

      System.out.println("moveIntake: " + desiredSpeed);
    });
  } 

  public Command intakeIn(){
    return run(()->{
      intakeMax.set(intakeSpeed);
    });
  } 

  public Command intakeOut(){
    return run(()->{
                  HotdogMotor.set(1);

      intakeMax.set(-intakeSpeed);
    });
  } 

  public Command intakeStop(){
    return run(()->{
                  HotdogMotor.set(0);

      intakeMax.set(0);
    });
  } 


  public Command Swap(){
      /*
       * Run only once when the A button is down.
       * 
       * Swaps the direction the intake mechanism is moving.
       */
    return run( () -> {
        
      isTopPressed = m_toplimitswitch.get();
      isBottomPressed = m_bottomlimitswitch.get();
       
      if (isBottomPressed && isIntakeOut){
        isMovingOut = false;
      }

      if (isTopPressed && !isIntakeOut){
        isMovingOut = true;
      }
             

      if (isTopPressed){
        System.out.println("STOP UP");
        isIntakeOut = false;
        System.out.println("isIntakeOut:" + isIntakeOut);
      }

      if (isBottomPressed){
        System.out.println("STOP DOWN");
        isIntakeOut = true;          
      }
    });
  }
  public Command MoveIntake(){
    /* 
     * Always running
     * Moves the intake
     * and stops it with limits
     */
    return run(()->{
      
            if (isMovingOut){
              intakeMoverSpeed = -intakeMoverSpeedConstant;
            }
      
            if (!isMovingOut){
              intakeMoverSpeed = intakeMoverSpeedConstant;
            }
      
            if (isTopPressed){
              intakeMoverSpeed = MathUtil.clamp(intakeMoverSpeed,-1,0);
            }
            if (isBottomPressed){
              intakeMoverSpeed = MathUtil.clamp(intakeMoverSpeed,0,1);
            }
      
            intakeMoverMax.set(intakeMoverSpeed);
            System.out.println (intakeMoverSpeed);
    
      });
    }

  public Command moveHotDog(boolean reverse){
    return run(()->{
      double desiredSpeed = 1;
      if(reverse){
        desiredSpeed = -desiredSpeed;
      }
      
      HotdogMotor.set(desiredSpeed);
    });
  } 

  public Command stopHotDog(){
    return run(()->{
      HotdogMotor.set(0);
    });
  } 
}
