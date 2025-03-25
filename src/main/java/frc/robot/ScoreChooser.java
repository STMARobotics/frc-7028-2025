package frc.robot;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Tracks the selected coral scoring level, and sends it over NT for the dashboard.
 */
public class ScoreChooser {

  private final NetworkTable scoreChooserTable = NetworkTableInstance.getDefault().getTable("ScoreChooser");
  private final BooleanPublisher level1Publisher = scoreChooserTable.getBooleanTopic("Level 1").publish();
  private final BooleanPublisher level3Publisher = scoreChooserTable.getBooleanTopic("Level 3").publish();
  private final BooleanPublisher level4Publisher = scoreChooserTable.getBooleanTopic("Level 4").publish();

  private int selectedLevel = 4;

  /**
   * Constructs a new ScoreChooser
   */
  public ScoreChooser() {
    selectLevel4();
  }

  /**
   * Selects level 1 for coral scoring
   */
  public void selectLevel1() {
    selectLevel(1);
  }

  /**
   * Selects level 3 for coral scoring
   */
  public void selectLevel3() {
    selectLevel(3);
  }

  /**
   * Selects level 4 for coral scoring
   */
  public void selectLevel4() {
    selectLevel(4);
  }

  /**
   * Checks if level 1 is selected
   * 
   * @return true if level 1 is selected, otherwise false
   */
  public boolean isLevel1Selected() {
    return 1 == selectedLevel;
  }

  /**
   * Checks if level 3 is selected
   * 
   * @return true if level 3 is selected, otherwise false
   */
  public boolean isLevel3Selected() {
    return 3 == selectedLevel;
  }

  /**
   * Checks if level 4 is selected
   * 
   * @return true if level 4 is selected, otherwise false
   */
  public boolean isLevel4Selected() {
    return 4 == selectedLevel;
  }

  private void selectLevel(int level) {
    selectedLevel = level;
    level1Publisher.set(1 == selectedLevel);
    level3Publisher.set(3 == selectedLevel);
    level4Publisher.set(4 == selectedLevel);
  }

}