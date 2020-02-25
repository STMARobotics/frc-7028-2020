package frc.robot;

/**
 * OperatorConsole
 */
public enum OperatorConsoleButton {

  LeftLeftButton(1),
  LeftRightButton(2),
  LeftTopButton(3),
  LeftBottomButton(4),
  RightLeftButton(5),
  RightRightButton(6),
  RightTopButton(7),
  JoystickUp(8),
  JoystickDown(9),
  GuardedSwitch(10);

  public final int value;

  private OperatorConsoleButton(int value) {
    this.value = value;
  }
  
}