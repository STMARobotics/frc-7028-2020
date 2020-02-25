package frc.robot;

/**
 * OperatorConsole
 */
public enum OperatorConsoleButton {

  LeftLeftButton(0),
  LeftRightButton(1),
  LeftTopButton(2),
  LeftBottomButton(3),
  RightLeftButton(4),
  RightRightButton(5),
  RightTopButton(6),
  JoystickUp(7),
  JoystickDown(8),
  GuardedSwitch(9);

  public final int value;

  private OperatorConsoleButton(int value) {
    this.value = value;
  }
  
}