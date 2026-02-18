package frc.robot.subsystems.swervedrive.TurretSubsystem;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;

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
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.spark.SparkMax;


public class Turret_Shoot {
    TalonFX shooter = new TalonFX(Constants.ShooterConstants.ShooterMotorID);
    SparkMax feeder = new SparkMax(Constants.ShooterConstants.FeederMotorID,  com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    XboxController controller = new XboxController(0);

     public void shooter(){
        if (controller.getRightTriggerAxis() > 0.1){
            shooter.set(0.5);
            feeder.set(0.5);
        } else {
            shooter.set(0);
            feeder.set(0);
        }
    }
}