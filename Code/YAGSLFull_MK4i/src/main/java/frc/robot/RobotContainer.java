// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SimElevator;
import frc.robot.subsystems.SimHand;
import frc.robot.subsystems.SimIntake;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

  final CommandXboxController Cmdriver = new CommandXboxController(0);
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/MK4i"));

  private final SendableChooser<Command> autoChooser;

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> -Cmdriver.getLeftY(),
                                                                () -> -Cmdriver.getLeftX())
                                                            .withControllerRotationAxis(() -> (-Cmdriver.getRightX()) * 0.8)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  private SimIntake mSimIntake = new SimIntake(20, 1.5, 3);
  private SimElevator mSimElevator = new SimElevator(400, 1.5);
  private SimHand mSimHand = new SimHand(-180, 180, 1.5, 3);

  public RobotContainer() {
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto", autoChooser);
  }

  private void configureBindings() {
    Command driveMode = drivebase.driveFieldOriented(driveAngularVelocity);

    drivebase.setDefaultCommand(driveMode);
    /*
    Cmdriver.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));

    Cmdriver.a().onTrue(Commands.runOnce(() -> mSimIntake.setPosition(20, 3)));
    Cmdriver.a().onTrue(Commands.runOnce(() -> mSimIntake.setVelocity(1)));
    Cmdriver.b().onTrue(Commands.runOnce(() -> mSimIntake.setPosition(0, 3)));
    Cmdriver.b().onTrue(Commands.runOnce(() -> mSimIntake.setVelocity(0)));
    Cmdriver.back().onTrue(Commands.runOnce(() -> mSimIntake.setPosition(0, 3)));
    Cmdriver.back().onTrue(Commands.runOnce(() -> mSimIntake.setVelocity(0)));

    Cmdriver.povUp().onTrue(mSimElevator.CMDsetPosition(400, 3));
    Cmdriver.povDown().onTrue(mSimElevator.CMDsetPosition(0, 3));
    Cmdriver.back().onTrue(Commands.runOnce(() -> mSimElevator.setPosition(0, 3)));

    Cmdriver.povLeft().onTrue(Commands.runOnce(() -> mSimHand.setAngle(-45, 3)));
    Cmdriver.povRight().onTrue(Commands.runOnce(() -> mSimHand.setAngle(45, 3)));

    Cmdriver.y().onTrue(Commands.runOnce(() -> mSimHand.setVelocity(2)));
    Cmdriver.x().onTrue(Commands.runOnce(() -> mSimHand.setVelocity(-2)));
    Cmdriver.b().onTrue(Commands.runOnce(() -> mSimHand.setVelocity(0)));
    Cmdriver.back().onTrue(Commands.runOnce(() -> mSimHand.setAngle(0, 3)));
    Cmdriver.back().onTrue(Commands.runOnce(() -> mSimHand.setVelocity(0)));
    */

    Cmdriver.a().whileTrue(followPath("Aula"));

    Cmdriver.y().whileTrue(drivebase.driveToPose(new Pose2d(4,4, new Rotation2d())));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  /**
   * Executa um path desejado.
   * @param pathName String do path que deseja executar.
   * @return movimentos e ações presentes no path.
   */
  private Command followPath(String pathName) {
      return Commands.defer(() -> {
          try {
              PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
              return AutoBuilder.followPath(path);
          } catch (Exception e) {
              DriverStation.reportError("path Error: " + pathName, e.getStackTrace());
              return Commands.none();
          }
      }, java.util.Set.of(drivebase));
  }

  /**
   * Ativa o comando alvo, quando a condição é verdadeira.
   * @param null
   */
  private void onCommandCondition(BooleanSupplier condition, Command command) {
      new Trigger(condition).onTrue(command);
  }

  /**
   * Matém um comando ativo, enquanto a condição é verdade.
   * @param null
   */
  private void whileCommandCondition(BooleanSupplier condition, Command command) {
      new Trigger(condition).whileTrue(command);
  }
}
