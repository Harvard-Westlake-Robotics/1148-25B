package frc.robot.subsystems.drive;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.networktables.*;
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
  private boolean isAuto;

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
    isAuto = true;

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
    this.isAuto = isAuto;
    isAutoBooleanPublisher.set(isAuto);
  }

  public PathPlannerPath getSelectedReefPath() {
    return paths.get("" + (char) (getTeleopBranch() + 'A'));
  }
}
