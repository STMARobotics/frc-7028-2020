package frc.robot.commands;

import static org.mockito.Mockito.inOrder;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.Mock;
import org.mockito.junit.MockitoJUnitRunner;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Unit tests for RunIntakeCommand
 */
@RunWith(MockitoJUnitRunner.class)
public class RunIntakeCommandTest {

  @Mock
  private IntakeSubsystem intake;

  private RunIntakeCommand intakeCommand;
  private CommandScheduler commandScheduler;

  @Before
  public void setUp() {
    commandScheduler = CommandScheduler.getInstance();
    intakeCommand = spy(new RunIntakeCommand(intake));
    when(intakeCommand.runsWhenDisabled()).thenReturn(true);
  }

  @Test
  public void testSpinUp() {

    commandScheduler.schedule(intakeCommand);

    commandScheduler.run();
    commandScheduler.run();

    verify(intake, times(2)).intake();;
  }
  
  @Test
  public void testStop() {

    commandScheduler.schedule(intakeCommand);

    commandScheduler.run();
    commandScheduler.cancel(intakeCommand);

    var inOrder = inOrder(intake);
    inOrder.verify(intake).intake();;
    inOrder.verify(intake).stopIntake();
  }

}