// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.TurretSubsystem.Turret_Hood;
import frc.robot.subsystems.swervedrive.TurretSubsystem.Turret_Shoot;
import frc.robot.subsystems.swervedrive.LEDs;


import java.io.File;
import java.util.function.BooleanSupplier;

import swervelib.SwerveInputStream;
import frc.robot.subsystems.swervedrive.ClimberSubsystem;
import frc.robot.subsystems.swervedrive.IntakeFlipperSubsystem;
import frc.robot.subsystems.swervedrive.IntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{


  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController operatorXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/Kraken"));
  private final SendableChooser<Command> autoChooser;

  private final Turret_Hood hoodSubsystem = new Turret_Hood();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final IntakeFlipperSubsystem intake2 = new IntakeFlipperSubsystem();
  private final Turret_Shoot shootSubsystem = new Turret_Shoot(intake);

 // private final LEDs led = new LEDs();
  private final ClimberSubsystem Climber = new ClimberSubsystem();
  
 

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
                                                                      // Derive the heading axis with math!
                                                                    SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                    .withControllerHeadingAxis(() -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
                                                                    () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
                                                                    .headingWhile(true)
                                                                    .translationHeadingOffset(true)
                                                                    .translationHeadingOffset(Rotation2d.fromDegrees(0));
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("shoot", shootSubsystem.shoot(false));
    autoChooser = AutoBuilder.buildAutoChooser();

    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */

  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    if (DriverStation.isTest()) {
      shootSubsystem.setDefaultCommand(shootSubsystem.stopShooter());
      hoodSubsystem.setDefaultCommand(hoodSubsystem.StopHood());
      intake.setDefaultCommand(intake.intakeStop());
      intake.setDefaultCommand(intake.stopHotDog());
      Climber.setDefaultCommand(Climber.ClimbStop());
      intake2.setDefaultCommand(intake2.MoveToDesiredState());

      testControls();
    } 
    else {
      TurretSubsystem();
      IntakeSubsystem();
      ClimberSubsystem();
      LEDs();

      System.out.println("ZeroReset1111");
      
      driverXbox.y().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
    
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("test1");
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
void LEDs() {
  //led.ColorChange(led.HubTimer()).repeatedly();
}

  void TurretSubsystem() {
    //shooter flywheels
    //tower motor
    //Hot dogrollers
    operatorXbox.rightTrigger()
      .onTrue(shootSubsystem.shoot(operatorXbox.b().getAsBoolean()))
      .onFalse(shootSubsystem.stopShooter());

    //Hood Motor
    operatorXbox.leftBumper()
      .onTrue(hoodSubsystem.MoveHood(operatorXbox.b().getAsBoolean()))
      .onFalse(hoodSubsystem.StopHood());
  }

  void IntakeSubsystem() {
    //Intake Roller Motor
    operatorXbox.leftTrigger()
      .whileTrue(intake.moveIntake(operatorXbox.b().getAsBoolean()))
      .whileFalse(intake.intakeStop());

    //Intake Flipper Motor
    //Debouce events faster than 0.2 seconds
    operatorXbox.x()
      .debounce(0.2)
      .onTrue(intake2.SwapDesiredState())
      .onFalse(intake2.MoveToDesiredState());

    operatorXbox.a()
      .onTrue(intake.moveHotDog(operatorXbox.b().getAsBoolean()))
      .onFalse(intake.stopHotDog());
  }
  
  void ClimberSubsystem() {
    // up on the D-Pad goes up
    operatorXbox.povUp()
      .onTrue(Climber.ClimbUp())
      .whileFalse(Climber.ClimbStop());

    // down on the D-Pad goes down
    operatorXbox.povDown()
      .onTrue(Climber.ClimbDown())
      .whileFalse(Climber.ClimbStop());
  }

  void testControls() {
    //Shooter Motors
    operatorXbox.rightTrigger()
      .onTrue(shootSubsystem.testShoot(operatorXbox.b().getAsBoolean()))
      .onFalse(shootSubsystem.stopShooter());
    
    //Tower Motor
    operatorXbox.leftTrigger()
      .onTrue(shootSubsystem.testTower(operatorXbox.b().getAsBoolean()))
      .onFalse(shootSubsystem.stopShooter());

    //Hot dog Motor
    operatorXbox.rightBumper()
      .onTrue(intake.moveHotDog(operatorXbox.b().getAsBoolean()))
      .onFalse(intake.stopHotDog());

    //Hood Motor
    operatorXbox.leftBumper()
      .onTrue(hoodSubsystem.MoveHood(operatorXbox.b().getAsBoolean()))
      .onFalse(hoodSubsystem.StopHood());

    //Intake Roller Motor
    operatorXbox.y()
      .onTrue(intake.moveIntake(operatorXbox.b().getAsBoolean()))
      .onFalse(intake.intakeStop());

    //Intake Flipper Motor
    operatorXbox.x()
      .onTrue(intake2.SwapDesiredState())
      .onFalse(intake2.MoveToDesiredState());

    //Climber Motor
    operatorXbox.a()
      .onTrue(Climber.Climb(operatorXbox.b().getAsBoolean()))
      .onFalse(Climber.ClimbStop());
  }

}
