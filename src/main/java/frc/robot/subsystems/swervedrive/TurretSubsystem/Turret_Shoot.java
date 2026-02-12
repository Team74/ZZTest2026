package frc.robot.subsystems.swervedrive.TurretSubsystem;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfigAccessor;

public class Turret_Shoot {
    TalonFX Shoot_Motor = new TalonFX(74); 
    DigitalInput testinput2026 = new DigitalInput(2);
    XboxController driveController = new XboxController(0);
    TalonFX armMotor = new TalonFX(3);
    //wheel spin
    public void setRPM(){

        if (driveController.getRightTriggerAxis()> 0 && testinput2026.get()){

           //TalonFX.limitSensorBottom.get() && AngularVelocityUnit.RPM > 40 && RPM < 30){
        //System.out.println("reach MAX speed");

        //RPM = 0;
        //}
        
    }

}
}