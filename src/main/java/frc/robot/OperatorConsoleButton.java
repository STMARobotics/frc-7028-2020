package frc.robot;

/**
 * OperatorConsole
 */
public enum OperatorConsoleButton {

  LeftLeftButton(13),
  LeftRightButton(14),
  LeftTopButton(7),
  LeftBottomButton(9),
  RightLeftButton(3),
  RightRightButton(4),
  RightTopButton(12),
  JoystickUp(2),
  JoystickDown(5),
  GuardedSwitch(11);

  public final int value;

  private OperatorConsoleButton(int value) {
    this.value = value;
  }
  
}