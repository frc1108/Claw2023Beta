// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SliderConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.drive.Drive;
//import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final Drive m_swerve = new Drive();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final ClawSubsystem m_claw = new ClawSubsystem();
  private final ExtenderSubsystem m_slider = new ExtenderSubsystem();
  //private final LEDSubsystem m_leds = new LEDSubsystem();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(
                                             OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(
                                             OIConstants.kOperatorControllerPort);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);

   /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

      NamedCommands.registerCommand("scoreCubeHigh", this.scoreCubeAutoCommand().alongWith(Commands.print("Cube Score")));
      NamedCommands.registerCommand("autoBalance", m_swerve.autoBalance());

      autoChooser.addDefaultOption("Nothing", Commands.none());
      autoChooser.addOption("SpeedBump",new PathPlannerAuto("SpeedBumpAuto"));
      autoChooser.addOption("Center Cube Back", this.scoreCubeAutoCommand().andThen(m_swerve.autoBalanceBackwards()));
      autoChooser.addOption("AutoBalance",m_swerve.autoBalance());
      autoChooser.addOption("High Cube",this.scoreCubeAutoCommand());

    // Configure the button bindings
    configureButtonBindings();

    

    // Configure default commands
    m_swerve.setDefaultCommand(
      new RunCommand(
        () -> m_swerve.drive(
          -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
          true, true),m_swerve));
    
    m_arm.setDefaultCommand(
      new RunCommand(
        () -> m_arm.set(-ArmConstants.kMaxArmSpeed*
          MathUtil.applyDeadband(m_operatorController.getRightY(),
          ArmConstants.kArmDeadband)),m_arm));

    m_slider.setDefaultCommand(
      new RunCommand(()->m_slider.set(
        SliderConstants.kMaxSliderSpeed*
        MathUtil.applyDeadband(m_operatorController.getLeftY(),
        SliderConstants.kSliderDeadband)),m_slider
    ));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // Drive to X Pattern (Right bumper)
    m_driverController.rightBumper().whileTrue(Commands.run(m_swerve::setX));

    // Gyro reset (A button)
    m_driverController.a().onTrue(Commands.runOnce(m_swerve::zeroHeading));

    // LED Pattern advanced (B button)
    //m_driverController.b().onTrue(Commands.runOnce(m_leds::nextPattern,m_leds));

    // Move the arm to 2 radians above horizontal when the 'A' button is pressed.
    m_operatorController.y().onTrue(m_arm.setArmGoalCommand(Units.degreesToRadians(30)));

    // m_operatorController.y().onTrue(m_slider.setSliderGoalCommand(-Units.inchesToMeters(0.5)));
    m_operatorController.b().onTrue(m_slider.setSliderGoalCommand(-Units.inchesToMeters(12)));

    // Elevator control on POV Up (up) & Down (down)
    m_operatorController.povUp().onTrue(m_elevator.upCommand());
    m_operatorController.povDown().onTrue(m_elevator.downCommand());

    // Claw control on button A (grip) & X (release)
    m_operatorController.a().onTrue(m_claw.gripCommand());
    m_operatorController.x().onTrue(m_claw.releaseCommand());

    // Request LED color change for human player stations
    // m_operatorController.povLeft().onTrue(Commands.runOnce(
    //                                       ()->m_leds.setConePattern(),m_leds));
    // m_operatorController.povRight().onTrue(Commands.runOnce(
    //                                       ()->m_leds.setCubePattern(),m_leds));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetAutoHeading() {
    m_swerve.zeroHeading();
  }
  
  public Command scoreCubeAutoCommand(){
    return Commands.sequence(
      Commands.print("Score auto cube"),
      m_claw.gripCommand(),
      Commands.waitSeconds(0.25),
      m_elevator.upCommand(),
      Commands.waitSeconds(1.35),
      m_arm.setArmGoalCommand(ArmConstants.kArmHighCubeOffsetRads),
      Commands.waitSeconds(0.3),
      m_slider.setSliderGoalCommand(SliderConstants.kSliderHighCubeMeters),
      Commands.waitSeconds(1),
      m_claw.releaseCommand(),
      Commands.waitSeconds(0.1),
      m_slider.setSliderGoalCommand(SliderConstants.kSliderStowMeters),
      Commands.waitSeconds(1),
      m_elevator.downCommand(),
      Commands.waitSeconds(0.1),
      m_arm.setArmGoalCommand(ArmConstants.kArmOffsetRads+Units.degreesToRadians(15))
    );
  }

  /**
   * 
   * @return Autos class for Robot heading reset and auto commands
   */
  public ArmSubsystem getArm() {
    return m_arm;
  }
  /**
   * 
   * @return Autos class for Robot heading reset and auto commands
   */
  public ExtenderSubsystem getSlider() {
    return m_slider;
  }
}
