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

public class IntakeFlipperSubsystem extends SubsystemBase{
  public enum eDesiredEndState {
    IN,
    OUT
  }
  public enum eCurrentState {
    IN_STOPPED,
    OUT_STOPPED,
    MOVING_IN,
    MOVING_OUT
  }

  eCurrentState currentState = eCurrentState.IN_STOPPED; 
  eDesiredEndState currentDesiredState = eDesiredEndState.IN; 

  SparkMax intakeMoverMax = new SparkMax(Constants.IntakeConstants.MoverMotorID, MotorType.kBrushless);
 
  DigitalInput m_toplimitswitch = new DigitalInput(0);
  DigitalInput m_bottomlimitswitch = new DigitalInput(1);

  double intakeMoverSpeedConstant = Constants.IntakeConstants.IntakeMoverSpeed;
  double desiredintakeMoverSpeed = 0;

  public Command SwapDesiredState(){
    return run( () -> {
      if(currentDesiredState == eDesiredEndState.IN) {
        currentDesiredState = eDesiredEndState.OUT;
      }
      else {
        currentDesiredState = eDesiredEndState.IN;
      }
      System.out.println("swap");
    });
  }

  public Command MoveToDesiredState(){
    return run( () -> {
      var isTopPressed = m_toplimitswitch.get();
      var isBottomPressed = m_bottomlimitswitch.get();

      //If both limits are pressed.... Prob should check the bot....
      if(isTopPressed && isBottomPressed) {
        //Stop all
        desiredintakeMoverSpeed = 0;
      }
      //If the top limit is hit but not the bottom
      else if(isTopPressed && !isBottomPressed) {
        if(currentDesiredState == eDesiredEndState.IN) {
          //Top is hit and you want to bring it back IN
          //Send 0
          desiredintakeMoverSpeed = 0;
          currentState = eCurrentState.IN_STOPPED;
        }
        if(currentDesiredState == eDesiredEndState.OUT) {
          //Top is hit and you want to bring it back OUT
          //Send neg value
          desiredintakeMoverSpeed = -intakeMoverSpeedConstant;
          currentState = eCurrentState.MOVING_OUT;
        }
      }
      //If the top limit is NOT hit but the bottom limit IS
      else if(!isTopPressed && isBottomPressed) {
        if(currentDesiredState == eDesiredEndState.IN) {
          //Bottom is hit so make sure to give motor a positive value.
          //Send pos val
          desiredintakeMoverSpeed = intakeMoverSpeedConstant;
          currentState = eCurrentState.MOVING_IN;
        }
        if(currentDesiredState == eDesiredEndState.OUT) {
          //Bottom is hit so make sure to give motor a positive value.
          //Send 0
          desiredintakeMoverSpeed = 0;
          currentState = eCurrentState.OUT_STOPPED;
        }
      }
      //If both limits are not hit.
      else {
        //Do nothing keep moving in the previous direction until one of the limits are tripped  
      }

      intakeMoverMax.set(desiredintakeMoverSpeed);
      System.out.println ("isTopPressed: " + isTopPressed + " isBottomPressed: " + isBottomPressed + " desiredintakeMoverSpeed: " + desiredintakeMoverSpeed + " currentState: " + currentState + " currentDesiredState: " + currentDesiredState);
    });
  }

  public Command Stop(){
    return run( () -> {
      intakeMoverMax.set(0);
    });
  }


}
