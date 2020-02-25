package frc.robot;

/**
 * OperatorConsole
 */
public enum OperatorConsoleButton {

  LeftThumbButton(8),
  RightThumbButton(6),
  IndexFingerButton(12),
  MiddleFingerButton(11),
  RingFingerButton(13),
  PinkyFingerButton(5),
  LeftJoystickButton(2),
  RightJoystickButton(3),
  LeftJoystickUp(7),
  LeftJoystickDown(9),
  RightJoystickUp(1),
  RightJoystickDown(4),
  GuardedButton(10);

  public final int value;

  private OperatorConsoleButton(int value) {
    this.value = value;
  }
  
}