package frc.robot.subsystems.swervedrive.TurretSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfigAccessor;

public class Turret_Hood extends SubsystemBase {
    SparkMax hood = new SparkMax(Constants.ShooterConstants.HoodMotorID,MotorType.kBrushless);
    //XboxController controller = new XboxController(1);
    double hoodSpeed = 0.25;
   
    public Command MoveHoodIn(){
        return run(()->{
            hood.set(-hoodSpeed);
        });
    }

    public Command MoveHoodOut(){
        return run(()->{
            hood.set(hoodSpeed);
        });
    }

    public Command StopHood(){
        return run(()->{
            hood.set(0);
        });
    }
}
