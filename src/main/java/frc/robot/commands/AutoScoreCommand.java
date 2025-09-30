package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmCommand.ScoringLevel;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.CoralIntake;
import frc.robot.subsystems.wrist.ArmWrist;
import frc.robot.subsystems.wrist.IntakeWrist;
import frc.robot.util.ArmKinematics;
import org.littletonrobotics.junction.Logger;

/**
 * Command to automatically score game pieces at specified heights. This command handles both the
 * elevator movement and drive positioning. It includes safety checks, timeouts, and error handling.
 */
public class AutoScoreCommand extends Command {
  // Constants for position and timing
  public static double POSITION_TOLERANCE = 0.02; // meters
  public static double ROTATION_TOLERANCE = 0.6; // degrees
  private static final double ARM_Y_TOLERANCE = 0.4; // meters
  private static final double SCORING_DELAY_TICKS = 3;
  private static final double SCORING_VELOCITY = 22.0; // meters per second
  // private static final double DEFAULT_VELOCITY = 6.0; // meters per second
  private static final double COMMAND_TIMEOUT = 10.0; // seconds

  // PID controller constants
  public static double kP = 18.7;
  public static double kD = 0.05;
  public static double THETA_PID_P = 9.9;
  public static double THETA_PID_D = 0.1;
  private static final double MAX_VELOCITY = 2.0;
  private static final double MAX_ACCELERATION = 2.75;
  private static final double MAX_ANGULAR_VELOCITY = 2.0;
  private static final double MAX_ANGULAR_ACCELERATION = 2.75;

  private final ScoringLevel level;
  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;
  private final Pose2d endPose;
  private int tickCounter = 0;
  private final Timer timeoutTimer;

  /**
   * Creates a new AutoScoreCommand with path following capability.
   *
   * @param level The scoring level to move to
   * @param path The path to follow to the scoring position
   */
  public AutoScoreCommand(ScoringLevel level, PathPlannerPath path) {
    this.addRequirements(
        CoralIntake.getInstance(),
        Elevator.getInstance(),
        path == null ? Drive.getInstance() : null);
    this.level = level;
    this.timeoutTimer = new Timer();

    // Initialize controllers with constants
    this.xController =
        new ProfiledPIDController(kP, 0, kD, new Constraints(MAX_VELOCITY, MAX_ACCELERATION));
    this.yController =
        new ProfiledPIDController(kP, 0, kD, new Constraints(MAX_VELOCITY, MAX_ACCELERATION));
    this.thetaController =
        new ProfiledPIDController(
            THETA_PID_P,
            0,
            THETA_PID_D,
            new Constraints(MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCELERATION));

    // Set up controllers
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    xController.setTolerance(POSITION_TOLERANCE);
    yController.setTolerance(POSITION_TOLERANCE);
    thetaController.setTolerance(Math.toRadians(ROTATION_TOLERANCE));

    // Safely get end pose from path
    if (path != null) {
      // Cancel any existing intake command
      if (CoralIntake.getInstance().getCurrentCommand() != null) {
        CoralIntake.getInstance().getCurrentCommand().cancel();
      }
      if (Drive.getInstance().getCurrentCommand() != null) {
        Drive.getInstance().getCurrentCommand().cancel();
      }

      this.endPose =
          new Pose2d(
              DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                  ? path.getPathPoses()
                      .get(path.getPathPoses().size() - 1)
                      .getTranslation()
                      .plus(new Translation2d(0.04, path.getGoalEndState().rotation()))
                  : path.flipPath()
                      .getPathPoses()
                      .get(path.getPathPoses().size() - 1)
                      .getTranslation()
                      .plus(new Translation2d(0.04, path.flipPath().getGoalEndState().rotation())),
              DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                  ? path.getGoalEndState().rotation()
                  : path.flipPath().getGoalEndState().rotation());

      Logger.recordOutput("RealOutputs/PIDEndPose", endPose);
    } else {
      this.endPose = null;
    }
  }

  @Override
  public void initialize() {
    // Reset controllers to current position
    xController.reset(Drive.getInstance().getPose().getX());
    yController.reset(Drive.getInstance().getPose().getY());
    thetaController.reset(Drive.getInstance().getPose().getRotation().getRadians());
    DriveConstants.setSdMultiplier(5.0);
    // Reset command state
    tickCounter = 0;
    timeoutTimer.reset();
    timeoutTimer.start();

    // Set initial positions and velocities
    RobotContainer.coralIntakeCommand.manual = true;
    CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
    RobotContainer.armCommand.setHeight(ScoringLevel.NEUTRAL);
  }

  @Override
  public void execute() {
    // Check for timeout
    if (timeoutTimer.get() > COMMAND_TIMEOUT) {
      this.cancel();
      return;
    }

    // Move arm to target height
    RobotContainer.armCommand.setHeight(level);

    // Handle path following if endPose is set
    if (endPose != null) {
      handlePathFollowing();
    } else {
      handleScoring();
    }
  }

  /** Handles the path following logic when an end pose is specified */
  private void handlePathFollowing() {
    Pose2d currentPose = Drive.getInstance().getPose();
    double distanceToTarget = currentPose.getTranslation().getDistance(endPose.getTranslation());
    double rotationError =
        Math.abs(currentPose.getRotation().getRadians() - endPose.getRotation().getRadians());

    if (distanceToTarget > POSITION_TOLERANCE
        || rotationError > Math.toRadians(ROTATION_TOLERANCE)) {
      Logger.recordOutput("RealOutputs/x_error", xController.getPositionError());
      Logger.recordOutput("RealOutputs/y_error", yController.getPositionError());
      Logger.recordOutput("RealOutputs/theta_error", thetaController.getPositionError());
      // Move to target position
      ChassisSpeeds speeds =
          new ChassisSpeeds(
              LinearVelocity.ofBaseUnits(
                  xController.calculate(currentPose.getX(), endPose.getX()), MetersPerSecond),
              LinearVelocity.ofBaseUnits(
                  yController.calculate(currentPose.getY(), endPose.getY()), MetersPerSecond),
              AngularVelocity.ofBaseUnits(
                  thetaController.calculate(
                      currentPose.getRotation().getRadians(), endPose.getRotation().getRadians()),
                  RadiansPerSecond));
      Drive.getInstance()
          .runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds, Drive.getInstance().getPose().getRotation()));
    } else {
      handleScoring();
    }
  }

  /** Handles the scoring logic once in position */
  private void handleScoring() {
    Drive.getInstance().stop();

    double currentHeight =
        ArmKinematics.getCurrentPose(
                new ArmKinematics.JointPose(
                    Rotation2d.fromRotations(ArmWrist.getInstance().getWristPosition()),
                    Meters.of(Elevator.getInstance().getExtention()), // L
                    Rotation2d.fromRotations(IntakeWrist.getInstance().getWristPosition())))
            .y()
            .magnitude();

    double targetHeight = WristConstants.getTargetY(level).magnitude();

    if (Math.abs(targetHeight - currentHeight) < ARM_Y_TOLERANCE || currentHeight > targetHeight) {
      if (tickCounter >= SCORING_DELAY_TICKS) {
        CoralIntake.getInstance()
            .setVelocity(LinearVelocity.ofBaseUnits(SCORING_VELOCITY, MetersPerSecond));
      } else {
        tickCounter++;
      }
    } else {
      CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Reset robot state
    if (CoralIntake.getInstance().hasCoralHotDog() || CoralIntake.getInstance().hasCoralBurger()) {
      CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
    } else {
      CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(4, MetersPerSecond));
    }
    RobotContainer.coralIntakeCommand.manual = false;
    RobotContainer.armCommand.setHeight(ScoringLevel.NEUTRAL);
    Drive.getInstance().stop();
    DriveConstants.setSdMultiplier(1);

    if (timeoutTimer.get() > COMMAND_TIMEOUT) {
      System.err.println("AutoScoreCommand ended due to timeout");
    }
  }

  @Override
  public boolean isFinished() {
    return (!CoralIntake.getInstance().hasCoralHotDog()
            && !CoralIntake.getInstance().hasCoralBurger())
        || timeoutTimer.get() > COMMAND_TIMEOUT;
  }
}
