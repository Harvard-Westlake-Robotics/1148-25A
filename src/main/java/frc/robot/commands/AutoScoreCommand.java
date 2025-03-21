package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.ScoreCommand.ScoringLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.CoralIntake;

public class AutoScoreCommand extends Command {
  private ScoringLevel level;
  private double targetHeight = 0.0;
  private ProfiledPIDController xController;
  private ProfiledPIDController yController;
  private ProfiledPIDController thetaController;
  private Pose2d endPose;
  private int tickCounter = 0;

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
    Drive.getInstance().setLimeLightsActive(false);
    this.addRequirements(CoralIntake.getInstance(), Elevator.getInstance(), Drive.getInstance());
    if (CoralIntake.getInstance().getCurrentCommand() != null) {
      CoralIntake.getInstance().getCurrentCommand().cancel();
    }
    RobotContainer.coralIntakeCommand.setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
    this.level = level;
    this.xController = new ProfiledPIDController(3.0, 0, 0.3, new Constraints(3.0, 5.0));
    this.yController = new ProfiledPIDController(3.0, 0, 0.3, new Constraints(3.0, 5.0));
    this.thetaController = new ProfiledPIDController(5.0, 0, 0.4, new Constraints(4.0, 10.0));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    xController.setTolerance(0.01);
    yController.setTolerance(0.01);
    thetaController.setTolerance(0.05);
    this.endPose = path.getPathPoses().get(2);
    xController.reset(Drive.getInstance().getPose().getX());
    yController.reset(Drive.getInstance().getPose().getY());
    thetaController.reset(Drive.getInstance().getPose().getRotation().getRadians());
  }

  @Override
  public void initialize() {
    if (level == ScoringLevel.L1) {
      targetHeight = 15.5;
    } else if (level == ScoringLevel.L2) {
      targetHeight = 20.45;
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
    if (Drive.getInstance().getPose().getTranslation().getDistance(endPose.getTranslation()) > 0.02
        || Drive.getInstance().getRotation().getMeasure().isNear(endPose.getRotation().getMeasure(),
            Angle.ofBaseUnits(0.5, Degrees))) {
      Pose2d poseThisTick = Drive.getInstance().getPose();
      CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
      Drive.getInstance()
          .runVelocity(
              new ChassisSpeeds(
                  LinearVelocity.ofBaseUnits(
                      xController.calculate(poseThisTick.getX(), endPose.getX()),
                      MetersPerSecond),
                  LinearVelocity.ofBaseUnits(
                      yController.calculate(poseThisTick.getY(), endPose.getY()),
                      MetersPerSecond),
                  AngularVelocity.ofBaseUnits(
                      thetaController.calculate(poseThisTick.getRotation().getRadians(),
                          endPose.getRotation().getRadians()),
                      RadiansPerSecond)));
    } else {
      if ((Math.abs(targetHeight - Elevator.getInstance().getHeight()) < 1
          || Elevator.getInstance().getHeight() > targetHeight)) {
        if (tickCounter > 9)
          CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(40, MetersPerSecond));
        else
          tickCounter++;
      } else {
        CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.coralIntakeCommand.setVelocity(LinearVelocity.ofBaseUnits(4, MetersPerSecond));
    Elevator.getInstance().goToHeight(0);
    CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(4, MetersPerSecond));
    Drive.getInstance().stop();
    Drive.getInstance().setLimeLightsActive(true);
  }

  @Override
  public boolean isFinished() {
    return !CoralIntake.getInstance().hasCoral();
  }
}
