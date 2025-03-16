package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ScoreCommand.ScoringLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.CoralIntake;

public class AutoScoreCommand extends Command {
  private ScoringLevel level;
  private double targetHeight = 0.0;
  private PIDController xController;
  private PIDController yController;
  private PIDController thetaController;
  private Pose2d endPose;

  public AutoScoreCommand(ScoringLevel level) {
    this.addRequirements(CoralIntake.getInstance(), Elevator.getInstance());
    this.level = level;
    if (level == ScoringLevel.L1) {
      targetHeight = 15.5;
    } else if (level == ScoringLevel.L2) {
      targetHeight = 21.25;
    } else if (level == ScoringLevel.L3) {
      targetHeight = 32.68;
    } else if (level == ScoringLevel.L4) {
      targetHeight = 53.40;
    } else if (level == ScoringLevel.TOP_REMOVE) {
      targetHeight = 19.12;
    } else if (level == ScoringLevel.BOTTOM_REMOVE) {
      targetHeight = 7.60;
    } else {
      targetHeight = 0;
    }

    Elevator.getInstance().goToHeight(targetHeight);
  }

  public AutoScoreCommand(ScoringLevel level, PathPlannerPath path) {
    this.addRequirements(CoralIntake.getInstance(), Elevator.getInstance(), Drive.getInstance());
    this.level = level;
    this.xController = new PIDController(2.2, 0, 0.00);
    this.yController = new PIDController(2.2, 0, 0.00);
    this.thetaController = new PIDController(1.0, 0, 0.00);
    thetaController.enableContinuousInput(-180, 180);
    this.endPose = path.getPathPoses().get(path.getPathPoses().size() - 1);
  }

  @Override
  public void initialize() {
    if (level == ScoringLevel.L1) {
      targetHeight = 15.5;
    } else if (level == ScoringLevel.L2) {
      targetHeight = 20.85;
    } else if (level == ScoringLevel.L3) {
      targetHeight = 33.48;
    } else if (level == ScoringLevel.L4) {
      targetHeight = 53.40;
    } else if (level == ScoringLevel.TOP_REMOVE) {
      targetHeight = 19.12;
    } else if (level == ScoringLevel.BOTTOM_REMOVE) {
      targetHeight = 7.60;
    } else {
      targetHeight = 0;
    }
    CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
    Elevator.getInstance().goToHeight(targetHeight);
  }

  @Override
  public void execute() {
    Elevator.getInstance().goToHeight(targetHeight);
    System.out.println(targetHeight);
    // if
    // (endPose.getTranslation().getDistance(Drive.getInstance().getPose().getTranslation())
    // > 0.05
    // ||
    // endPose.getRotation().minus(Drive.getInstance().getPose().getRotation()).getDegrees()
    // > 0.5) {
    // CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(0,
    // MetersPerSecond));
    // Drive.getInstance()
    // .runVelocity(
    // new ChassisSpeeds(
    // LinearVelocity.ofBaseUnits(
    // -xController.calculate(endPose.getX() -
    // Drive.getInstance().getPose().getX()),
    // MetersPerSecond),
    // LinearVelocity.ofBaseUnits(
    // -yController.calculate(endPose.getY() -
    // Drive.getInstance().getPose().getY()),
    // MetersPerSecond),
    // AngularVelocity.ofBaseUnits(
    // thetaController.calculate(
    // endPose
    // .getRotation()
    // .minus(Drive.getInstance().getPose().getRotation())
    // .getDegrees()),
    // DegreesPerSecond)));
    // } else {
    if (Math.abs(targetHeight - Elevator.getInstance().getHeight()) < 1
        || Elevator.getInstance().getHeight() > targetHeight) {
      CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(40, MetersPerSecond));
    } else {
      CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
    }
    // }
  }

  @Override
  public void end(boolean interrupted) {
    Elevator.getInstance().goToHeight(0);
    CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(4, MetersPerSecond));
    Drive.getInstance().stop();
  }

  @Override
  public boolean isFinished() {
    return !CoralIntake.getInstance().hasCoral();
  }
}
