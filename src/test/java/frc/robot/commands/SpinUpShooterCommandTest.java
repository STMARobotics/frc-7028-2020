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
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Unit tests for SpinUpShooterCommand
 */
@RunWith(MockitoJUnitRunner.class)
public class SpinUpShooterCommandTest {

  private static final int DISTANCE = 150;

  @Mock
  private ShooterSubsystem shooter;

  private SpinUpShooterCommand spinUpShooter;
  private CommandScheduler commandScheduler;

  @Before
  public void setUp() {
    commandScheduler = CommandScheduler.getInstance();
    spinUpShooter = spy(new SpinUpShooterCommand(DISTANCE, shooter));
    when(spinUpShooter.runsWhenDisabled()).thenReturn(true);
  }

  @Test
  public void testSpinUp() {

    commandScheduler.schedule(spinUpShooter);

    commandScheduler.run();
    commandScheduler.run();

    verify(shooter, times(2)).prepareToShoot(DISTANCE);
  }
  
  @Test
  public void testStop() {

    commandScheduler.schedule(spinUpShooter);

    commandScheduler.run();
    commandScheduler.cancel(spinUpShooter);

    var inOrder = inOrder(shooter);
    inOrder.verify(shooter).prepareToShoot(DISTANCE);
    inOrder.verify(shooter).stopShooter();
  }

}