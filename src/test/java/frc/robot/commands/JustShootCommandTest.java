package frc.robot.commands;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.mockito.Mockito.inOrder;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.when;

import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.InOrder;
import org.mockito.Mock;
import org.mockito.junit.MockitoJUnitRunner;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * ShootCommandTest
 */
@RunWith(MockitoJUnitRunner.class)
public class JustShootCommandTest {

  private static final int DISTANCE = 120;

  private JustShootCommand justShootCommand;

  @Mock
  private IndexerSubsystem indexer;

  @Mock
  private ShooterSubsystem shooter;

  private CommandScheduler commandScheduler;

  @Before
  public void setUp() {
    commandScheduler = CommandScheduler.getInstance();
    justShootCommand = spy(new JustShootCommand(1, DISTANCE, shooter, indexer));
    when(justShootCommand.runsWhenDisabled()).thenReturn(true);
  }

  @Test
  public void testImmediateShoot() {

    when(shooter.isReadyToShoot()).thenReturn(true);
    when(indexer.isFull()).thenReturn(true, false);

    justShootCommand.schedule();
    commandScheduler.run();
    Timer timer = new Timer();
    timer.start();
    do {
      Timer.delay(.02);
      commandScheduler.run();
    } while(!timer.hasElapsed(ShooterConstants.SHOOT_TIME));
    commandScheduler.run();

    InOrder inOrder = inOrder(shooter, indexer);
    inOrder.verify(shooter).prepareToShoot(DISTANCE);
    inOrder.verify(shooter).isReadyToShoot();
    inOrder.verify(indexer).shoot();
    inOrder.verify(shooter).stopShooter();
    inOrder.verify(indexer).stopIndexer();
  }

  @Test
  public void testShootThree() {
    justShootCommand = spy(new JustShootCommand(3, DISTANCE, shooter, indexer));
    when(justShootCommand.runsWhenDisabled()).thenReturn(true);

    when(shooter.isReadyToShoot()).thenReturn(true, false, true, false, true);
    when(indexer.isFull()).thenReturn(true, false, true, false, true, false);

    justShootCommand.schedule();
    commandScheduler.run(); // Shoot
    commandScheduler.run(); // Wait for spin up
    commandScheduler.run(); // Shoot
    commandScheduler.run(); // Wait for spin up
    commandScheduler.run(); // Shoot
    Timer timer = new Timer();
    timer.start();
    do {
      Timer.delay(.02);
      commandScheduler.run();
    } while(!timer.hasElapsed(ShooterConstants.SHOOT_TIME));
    commandScheduler.run();

    InOrder inOrder = inOrder(shooter, indexer);
    inOrder.verify(shooter).prepareToShoot(DISTANCE);
    inOrder.verify(shooter).isReadyToShoot();
    inOrder.verify(indexer).shoot();
    inOrder.verify(shooter).prepareToShoot(DISTANCE);
    inOrder.verify(shooter).isReadyToShoot();
    inOrder.verify(indexer).shoot();
    inOrder.verify(shooter).prepareToShoot(DISTANCE);
    inOrder.verify(shooter).isReadyToShoot();
    inOrder.verify(indexer).shoot();
    inOrder.verify(shooter).stopShooter();
    inOrder.verify(indexer).stopIndexer();

    assertEquals(3, justShootCommand.getBallsShot());
  }

  @Test
  public void testSpinUp() {

    when(shooter.isReadyToShoot()).thenReturn(false, true); // Not at target until second call
    when(indexer.isFull()).thenReturn(true, false, true);

    justShootCommand.schedule();
    commandScheduler.run();
    assertFalse(justShootCommand.isFinished());
    commandScheduler.run();
    Timer timer = new Timer();
    timer.start();
    do {
      Timer.delay(.02);
      commandScheduler.run();
    } while(!timer.hasElapsed(ShooterConstants.SHOOT_TIME));
    commandScheduler.run();

    InOrder inOrder = inOrder(shooter, indexer);
    inOrder.verify(shooter).isReadyToShoot();
    inOrder.verify(shooter).isReadyToShoot();
    inOrder.verify(indexer).shoot();
    inOrder.verify(shooter).stopShooter();
    inOrder.verify(indexer).stopIndexer();
  }

}