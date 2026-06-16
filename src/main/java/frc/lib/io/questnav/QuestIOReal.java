package frc.lib.io.questnav;

import frc.robot.generated.questnav.QuestNav;

public class QuestIOReal implements QuestIO {
  private final QuestNav questNav = new QuestNav();

  @Override
  public void updateInputs(QuestIOInputs inputs) {
    inputs.connected = questNav.isConnected();

    inputs.uncorrectedResetToQuest = inputs.uncorrectedPose.minus(inputs.uncorrectedResetPose);
  }
}