// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

//import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
//import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Superstructure;

@SuppressWarnings("unused")
public final class Autos {

  private final DriveSubsystem m_swerve;
  private final Superstructure m_superS;
  private HashMap<String, Command> eventMap;
  //private final SwerveAutoBuilder autoBuilder;

  // Autonomous selector on dashboard
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private GenericEntry kAutoStartDelaySeconds;
  
  public Autos(DriveSubsystem swerve,Superstructure superS) {
    this.m_swerve = swerve;
    this.m_superS = superS;

     configureNamedCommands();

    // CONFIGURE AUTOBUILDER LAST
    AutoBuilder.configureHolonomic(
      m_swerve::getPose,
      m_swerve::resetPose,
      m_swerve::getRobotRelativeSpeeds,
      m_swerve::driveRobotRelative,
      new HolonomicPathFollowerConfig(
        new PIDConstants(AutoConstants.kPXController,0,0),
        new PIDConstants(AutoConstants.kPThetaController,0,0),
        4.8, // max speed in m/s
        Units.inchesToMeters(Math.sqrt(Math.pow(28.5, 2)+Math.pow(18.5,2))/2), // Radius in meters of 28.5 x 18.5 inch robot using a^2 +b^2 = c^2
        new ReplanningConfig()
      ),
      m_swerve
    );

       // Autonomous selector options
    kAutoStartDelaySeconds = Shuffleboard.getTab("Live")
                                         .add("Auto Delay", 0)
                                         .withWidget(BuiltInWidgets.kNumberSlider)
                                         .withProperties((Map.of("Min", 0, "Max", 10, "Block increment", 1)))
                                         .getEntry();
    
    autoChooser.setDefaultOption("Nothing", Commands.none());
    //autoChooser.addOption("StraightTurn", driveTurn());
    //autoChooser.addOption("StraightTurnSpeedBump", driveTurnSpeedBump());
    // autoChooser.addOption("CubeBalance", cubeBB());
    //autoChooser.addOption("SpeedBump",speedBump());
    autoChooser.addOption("SpeedBump",newSpeedBump());
    //autoChooser.addOption("BigSpeedBump",bigSpeedBump());
    autoChooser.addOption("Center Cube Back", cubeCenterBackwards());
    autoChooser.addOption("AutoBalance",m_swerve.autoBalance());
    autoChooser.addOption("High Cube",m_superS.scoreCubeAutoCommand());

    SmartDashboard.putData("Auto Chooser",autoChooser);
  }

     public Command none() {
      return Commands.none();
    }
 
    // public Command cubeBB() {
    //   return new PathPlannerAuto("Cube Balance");
    // }
    
    public Command newSpeedBump() {
      return new PathPlannerAuto("SpeedBumpAuto");
    }

    // public Command speedBump() {
    //   return autoBuilder.fullAuto(PathPlanner.loadPathGroup("Speedbump",
    //   new PathConstraints(4, 3)));      
    // }

    // public Command driveTurn() {
    //   return autoBuilder.fullAuto(PathPlanner.loadPathGroup("StraightTurn",
    //   new PathConstraints(3, 2)));      
    // }

    // public Command driveTurnSpeedBump() {
    //   return autoBuilder.fullAuto(PathPlanner.loadPathGroup("StraightTurnSpeedBump",
    //   new PathConstraints(4, 3)));      
    // }

    // public Command bigSpeedBump() {
    //   return autoBuilder.fullAuto(PathPlanner.loadPathGroup("BigSpeedBump",
    //   new PathConstraints(0.8, 2)));      
    // }

    public Command cubeCenter() {
      return m_superS.scoreCubeAutoCommand().andThen(m_swerve.autoBalance());
    }
    
    public Command cubeCenterBackwards() {
      return m_superS.scoreCubeAutoCommand().andThen(m_swerve.autoBalanceBackwards());
    }
  
    // private HashMap<String, Command> buildEventMap() {
    //   return new HashMap<>(
    //       Map.ofEntries(
    //           Map.entry("scoreCubeHigh", m_superS.scoreCubeAutoCommand().alongWith(Commands.print("Cube Score"))),
    //           //Map.entry("scoreCubeHigh", Commands.print("Cube Score")),
    //           //Commands.print("Cube Score")),
    //           Map.entry("autoBalance", m_swerve.autoBalance())));
    //           // Commands.print("Auto Balance"))));
    // }

    private void configureNamedCommands() {
      NamedCommands.registerCommand("scoreCubeHigh", m_superS.scoreCubeAutoCommand().alongWith(Commands.print("Cube Score")));
      NamedCommands.registerCommand("autoBalance", m_swerve.autoBalance());
    }

      /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.sequence(
      Commands.waitSeconds(kAutoStartDelaySeconds.getDouble(0)),
      autoChooser.getSelected());
  }

  public void resetAutoHeading() {
    m_swerve.zeroHeading();
  }
  }