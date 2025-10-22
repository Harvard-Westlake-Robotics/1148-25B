package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmCommand.ScoringLevel;
import frc.robot.commands.AutoScoreCommand;
import frc.robot.commands.TeleopCommand;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.intake.CoralIntake;
import java.util.HashMap;

public class NetworkCommunicator {
  private static NetworkCommunicator instance;
  private NetworkTableInstance ntInst;
  private StringArraySubscriber autoSub;
  private IntegerSubscriber teleopSubBranch;
  private IntegerSubscriber teleopSubStation;
  private IntegerSubscriber teleopSubHeight;
  private BooleanPublisher isAutoBooleanPublisher;
  private HashMap<String, PathPlannerPath> paths;

  public TeleopCommand teleopCommand;

  private NetworkCommunicator() {}

  public static NetworkCommunicator getInstance() {
    if (instance == null) {
      instance = new NetworkCommunicator();
    }
    return instance;
  }

  public void init() {
    paths = new HashMap<String, PathPlannerPath>();

    try {
      paths.put("A", PathPlannerPath.fromPathFile("Pathfinding A"));
      paths.put("B", PathPlannerPath.fromPathFile("Pathfinding B"));
      paths.put("C", PathPlannerPath.fromPathFile("Pathfinding C"));
      paths.put("D", PathPlannerPath.fromPathFile("Pathfinding D"));
      paths.put("E", PathPlannerPath.fromPathFile("Pathfinding E"));
      paths.put("F", PathPlannerPath.fromPathFile("Pathfinding F"));
      paths.put("G", PathPlannerPath.fromPathFile("Pathfinding G"));
      paths.put("H", PathPlannerPath.fromPathFile("Pathfinding H"));
      paths.put("I", PathPlannerPath.fromPathFile("Pathfinding I"));
      paths.put("J", PathPlannerPath.fromPathFile("Pathfinding J"));
      paths.put("K", PathPlannerPath.fromPathFile("Pathfinding K"));
      paths.put("L", PathPlannerPath.fromPathFile("Pathfinding L"));
      paths.put("S1", PathPlannerPath.fromPathFile("Bottom Source 1"));
      paths.put("S2", PathPlannerPath.fromPathFile("Bottom Source 2"));
      paths.put("S3", PathPlannerPath.fromPathFile("Bottom Source 3"));
      paths.put("S4", PathPlannerPath.fromPathFile("Top Source 1"));
      paths.put("S5", PathPlannerPath.fromPathFile("Top Source 2"));
      paths.put("S6", PathPlannerPath.fromPathFile("Top Source 3"));
      paths.put("P", PathPlannerPath.fromPathFile("Pathfinding Processor"));
    } catch (Exception e) {
      System.out.println(e);
    }

    ntInst = NetworkTableInstance.getDefault();

    NetworkTable table = ntInst.getTable("uidata");

    // Communication with touchscreen
    autoSub = table.getStringArrayTopic("autocommands").subscribe(new String[0]);
    teleopSubBranch = table.getIntegerTopic("teleopbranch").subscribe(1);
    teleopSubStation = table.getIntegerTopic("teleopstation").subscribe(1);
    teleopSubHeight = table.getIntegerTopic("teleopheight").subscribe(1);
    isAutoBooleanPublisher = table.getBooleanTopic("isauto").publish();
    isAutoBooleanPublisher.set(true);
  }

  public void close() {
    autoSub.close();
    teleopSubBranch.close();
    teleopSubStation.close();
  }

  public String[] getAutoCommands() {
    return autoSub.get();
  }

  public long getTeleopBranch() {
    return teleopSubBranch.get();
  }

  public long getTeleopStation() {
    return teleopSubStation.get();
  }

  public PathPlannerPath getSelectedSourcePath() {
    return paths.get("S" + getTeleopStation());
  }

  public void setIsAuto(boolean isAuto) {
    isAutoBooleanPublisher.set(isAuto);
  }

  public PathPlannerPath getSelectedReefPath() {
    // Convert from branch number to letter
    return paths.get("" + (char) (getTeleopBranch() + 'A'));
  }

  public ScoringLevel getSelectedHeight() {
    if (teleopSubHeight.get() == 1) {
      return ScoringLevel.L1;
    } else if (teleopSubHeight.get() == 2) {
      return ScoringLevel.L2;
    } else if (teleopSubHeight.get() == 3) {
      return ScoringLevel.L3;
    } else if (teleopSubHeight.get() == 4) {
      return ScoringLevel.L4;
    } else return ScoringLevel.NEUTRAL;
  }

  public TeleopCommand getTeleopCommand() {
    if (teleopCommand == null) {
      teleopCommand = new TeleopCommand();
    }
    return teleopCommand;
  }

  public Command getCustomAuto() {
    String[] autoCommands = getAutoCommands();
    // Default command if nothing is selected -> go to neutral position
    if (autoCommands.length == 0) {
      return new PathPlannerAuto(Commands.none());
    } else {
      Command auto =
          new InstantCommand(
              () -> {
                RobotContainer.armCommand.setHeight(ScoringLevel.NEUTRAL);
              });
      // Command scheduler- adds each selected auto station to the auton
      for (int i = 0; i < autoCommands.length; i++) {
        // If selected command is a source command
        if (autoCommands[i].charAt(0) == 'S') {
          auto =
              auto.andThen(
                      // Go to selected source
                      AutoBuilder.pathfindThenFollowPath(
                          paths.get(autoCommands[i]), DriveConstants.PP_CONSTRAINTS))
                  // Source Intake
                  .andThen(
                      new InstantCommand(
                              () -> {
                                RobotContainer.armCommand.setHeight(ScoringLevel.SOURCE_CORAL);
                                RobotContainer.coralIntakeCommand.manual = true;
                                RobotContainer.coralIntakeCommand.velocity = MetersPerSecond.of(6);
                              })
                          .until(() -> CoralIntake.getInstance().hasCoralHotDog()));
          // If selected command is a reef command
        } else {
          ScoringLevel level = ScoringLevel.NEUTRAL;
          if (autoCommands[i].charAt(2) == '1') {
            level = ScoringLevel.L1;
          } else if (autoCommands[i].charAt(2) == '2') {
            level = ScoringLevel.L2;
          } else if (autoCommands[i].charAt(2) == '3') {
            level = ScoringLevel.L3;
          } else if (autoCommands[i].charAt(2) == '4') {
            level = ScoringLevel.L4;
          }
          auto =
              auto.andThen(
                      AutoBuilder.pathfindThenFollowPath(
                          paths.get("" + autoCommands[i].charAt(0)), DriveConstants.PP_CONSTRAINTS))
                  // 0.1 second delay
                  .andThen(new Command() {}.withTimeout(0.1))
                  .andThen(
                      // Run AutoScore Command
                      new AutoScoreCommand(
                          level, paths.get("" + (char) (autoCommands[i].charAt(0)))));
        }
      }
      return auto;
    }
  }
}
