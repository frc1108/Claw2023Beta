// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SliderConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
//import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Superstructure;
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
  private final DriveSubsystem m_swerve = new DriveSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final ClawSubsystem m_claw = new ClawSubsystem();
  private final ExtenderSubsystem m_slider = new ExtenderSubsystem();
  private final Superstructure m_superS = new Superstructure(m_arm, m_slider, m_elevator, m_claw);
  private final Autos m_autos = new Autos(m_swerve, m_superS);
  //private final LEDSubsystem m_leds = new LEDSubsystem();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(
                                             OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(
                                             OIConstants.kOperatorControllerPort);

   /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
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
   * 
   * @return Autos class for Robot heading reset and auto commands
   */
  public Autos getAutos() {
    return m_autos;
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
