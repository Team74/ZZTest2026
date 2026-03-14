package frc.robot.subsystems.swervedrive.TurretSubsystem;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.swervedrive.IntakeSubsystem;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.units.AngularVelocityUnit;

import static edu.wpi.first.units.Units.RPM;

import java.lang.ModuleLayer.Controller;

import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Turret_Shoot extends SubsystemBase {
  TalonFX shooterMotor = new TalonFX(Constants.ShooterConstants.ShooterMotorID);
  TalonFX shooterMotor2 = new TalonFX(Constants.ShooterConstants.ShooterMotor2ID);
  SparkMax towerMotor = new SparkMax(Constants.ShooterConstants.TowerMotorID, MotorType.kBrushless); 
 
  double currentRPS_Shooter = shooterMotor.getVelocity().getValueAsDouble();

  CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs()
    .withSupplyCurrentLimit(Constants.ShooterConstants.SupplyCurrentLimit)
    .withSupplyCurrentLimitEnable(Constants.ShooterConstants.SupplyCurrentLimitEnable)
    .withStatorCurrentLimit(Constants.ShooterConstants.StatorCurrentLimit)
    .withStatorCurrentLimitEnable(Constants.ShooterConstants.StatorCurrentLimitEnable);

  Slot0Configs slot0Configs = new Slot0Configs()
    .withKS(Constants.ShooterConstants.KS)
    .withKV(Constants.ShooterConstants.KV)
    .withKP(Constants.ShooterConstants.KP)
    .withKI(Constants.ShooterConstants.KI)
    .withKD(Constants.ShooterConstants.KD);

  TalonFXConfiguration toConfigure = new TalonFXConfiguration()
    .withCurrentLimits(m_currentLimits)
    .withSlot0(slot0Configs);

  VelocityVoltage m_velocityVoltage = new VelocityVoltage(0)
    .withSlot(0);

  double hoodSpeed = 0.25;

  Follower thign = new Follower(6, MotorAlignmentValue.Opposed);

  private final IntakeSubsystem intakeSubsystem;

  public Turret_Shoot(IntakeSubsystem intakeSubsystem) {
    shooterMotor.getConfigurator().apply(toConfigure);
    shooterMotor.setNeutralMode(NeutralModeValue.Coast);
    shooterMotor2.getConfigurator().apply(toConfigure);
    shooterMotor2.setNeutralMode(NeutralModeValue.Coast);

    this.intakeSubsystem = intakeSubsystem;
    shooterMotor2.setControl(thign);
  }

  public Command shoot(boolean reverse){
    return run(()->{
      currentRPS_Shooter = shooterMotor.getVelocity().getValueAsDouble();
      System.out.println(currentRPS_Shooter);

      var desiredSpeed = Constants.ShooterConstants.shooterDesiredRPS;
      if(reverse){
        desiredSpeed = -desiredSpeed;
      }        

      var request = new VelocityVoltage(0).withSlot(0);
      shooterMotor.setControl(request.withVelocity(desiredSpeed).withFeedForward(0.5));
      if (currentRPS_Shooter <= (desiredSpeed * 0.75) | reverse) {
        towerMotor.set(-desiredSpeed);
      }
     
    });
  } 

  public Command testShoot(boolean reverse){
    return run(()->{
      var desiredSpeed = Constants.ShooterConstants.shooterDesiredRPS;
      if(reverse){
        desiredSpeed = -desiredSpeed;
      }

      var request = new VelocityVoltage(0).withSlot(0);
      shooterMotor.setControl(request.withVelocity(desiredSpeed).withFeedForward(0.5));
      shooterMotor2.setControl(request.withVelocity(-desiredSpeed).withFeedForward(0.5));
      System.out.println("shoot" + desiredSpeed);
    });
  } 

  public Command testElevatorUp(){
    return run(()->{
      towerMotor.set(-0.5);
    });
  } 

  public Command testElevatorDown(){
    return run(()->{
      towerMotor.set(0.5);
    });
  } 
  public Command testElevatorStop(){
    return run(()->{
      towerMotor.set(0);
    });
  } 

   public Command reverseShoot(){
    return run(()->{
      currentRPS_Shooter = shooterMotor.getVelocity().getValueAsDouble();
        
      var request = new VelocityVoltage(0).withSlot(0);
      shooterMotor.setControl(request.withVelocity(Constants.ShooterConstants.desiredRPS).withFeedForward(0.5));
      shooterMotor2.setControl(thign);

      if (currentRPS_Shooter >= (Constants.ShooterConstants.shooterDesiredRPS * -0.75)) {
        towerMotor.set(Constants.ShooterConstants.shooterDesiredRPS);
      }
     
    });
  } 

  public Command stopShooter(){
    return run(()->{
      var request = new VelocityVoltage(0).withSlot(0);
      shooterMotor.setControl(request.withVelocity(0));
      shooterMotor2.setControl(request.withVelocity(0));     
      towerMotor.set(0);
    });
  } 

  public Command testTower(boolean reverse){
    return run(()->{
      var desiredSpeed = Constants.ShooterConstants.shooterDesiredRPS;
      if(reverse){
        desiredSpeed = -desiredSpeed;
      }
      towerMotor.set(desiredSpeed);    
    });
  } 
}
