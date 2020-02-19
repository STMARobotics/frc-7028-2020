package frc.robot.commands;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.AdditionalMatchers.gt;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.inOrder;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.verifyNoMoreInteractions;
import static org.mockito.Mockito.when;

import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.InOrder;
import org.mockito.Mock;
import org.mockito.junit.MockitoJUnitRunner;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * ShootCommandTest
 */
@RunWith(MockitoJUnitRunner.class)
@Ignore("Sadly, the test is out of date")
public class ShootCommandTest {

  private ShootCommand shootCommand;

  @Mock
  private LimelightSubsystem highLimelight;

  @Mock
  private LimelightSubsystem lowLimelight;

  @Mock
  private DriveTrainSubsystem drivetrain;

  @Mock
  private IndexerSubsystem indexer;

  @Mock
  private ShooterSubsystem shooter;

  private CommandScheduler commandScheduler;

  @Before
  public void setUp() {
    commandScheduler = CommandScheduler.getInstance();
    shootCommand = spy(new ShootCommand(shooter, indexer, highLimelight, lowLimelight, drivetrain));
    when(shootCommand.runsWhenDisabled()).thenReturn(true);
  }

  @Test
  public void testImmediateShoot() {

    var distanceToTarget = 1d;
    when(highLimelight.getTargetAcquired()).thenReturn(true);
    when(highLimelight.getTargetX()).thenReturn(0d);
    when(shooter.isReadyToShoot()).thenReturn(true);
    when(highLimelight.getMaxX()).thenReturn(LimeLightConstants.TARGET_X_MAX);
    when(highLimelight.getDistanceToTarget()).thenReturn(distanceToTarget);

    shootCommand.schedule();
    commandScheduler.run();

    InOrder inOrder = inOrder(shooter, indexer, drivetrain);
    inOrder.verify(shooter).prepareToShoot(distanceToTarget);
    inOrder.verify(drivetrain).arcadeDrive(0.0, -0.0, false);
    inOrder.verify(shooter).isReadyToShoot();
    inOrder.verify(indexer).shoot();
    inOrder.verify(shooter).stopShooter();
    inOrder.verify(indexer).stopIndexer();
    inOrder.verify(drivetrain).stop();

    assertTrue(shootCommand.isFinished());
  }

  @Test
  public void testLowLimelightImmediateShoot() {

    var distanceToTarget = 1d;
    when(highLimelight.getTargetAcquired()).thenReturn(false);
    when(lowLimelight.getTargetAcquired()).thenReturn(true);
    when(lowLimelight.getTargetX()).thenReturn(0d);
    when(shooter.isReadyToShoot()).thenReturn(true);
    when(lowLimelight.getMaxX()).thenReturn(LimeLightConstants.TARGET_X_MAX);
    when(lowLimelight.getDistanceToTarget()).thenReturn(distanceToTarget);

    shootCommand.schedule();
    commandScheduler.run();

    InOrder inOrder = inOrder(shooter, indexer, drivetrain);
    inOrder.verify(shooter).prepareToShoot(distanceToTarget);
    inOrder.verify(drivetrain).arcadeDrive(0.0, -0.0, false);
    inOrder.verify(shooter).isReadyToShoot();
    inOrder.verify(indexer).shoot();
    inOrder.verify(shooter).stopShooter();
    inOrder.verify(indexer).stopIndexer();
    inOrder.verify(drivetrain).stop();

    verify(lowLimelight).getTargetX();
    verify(lowLimelight).getDistanceToTarget();

    assertTrue(shootCommand.isFinished());
  }

  @Test
  public void testSpinUp() {

    var distanceToTarget = 1d;
    when(highLimelight.getTargetAcquired()).thenReturn(true);
    when(highLimelight.getTargetX()).thenReturn(0d);
    when(shooter.isReadyToShoot()).thenReturn(false, true); // Not at target until second call
    when(highLimelight.getMaxX()).thenReturn(LimeLightConstants.TARGET_X_MAX);
    when(highLimelight.getDistanceToTarget()).thenReturn(distanceToTarget);

    shootCommand.schedule();
    commandScheduler.run();
    assertFalse(shootCommand.isFinished());
    commandScheduler.run();

    InOrder inOrder = inOrder(shooter, indexer, drivetrain);
    inOrder.verify(shooter).prepareToShoot(distanceToTarget);
    inOrder.verify(shooter).isReadyToShoot();
    inOrder.verify(shooter).prepareToShoot(distanceToTarget);
    inOrder.verify(shooter).isReadyToShoot();
    inOrder.verify(indexer).shoot();
    inOrder.verify(shooter).stopShooter();
    inOrder.verify(indexer).stopIndexer();
    inOrder.verify(drivetrain).stop();

    assertTrue(shootCommand.isFinished());
  }

  @Test
  public void testTargeting() {

    var distanceToTarget = 1d;
    when(highLimelight.getTargetAcquired()).thenReturn(true);
    when(highLimelight.getTargetX()).thenReturn(5d, 0d); // Not at target until second call
    when(shooter.isReadyToShoot()).thenReturn(true);
    when(highLimelight.getMaxX()).thenReturn(LimeLightConstants.TARGET_X_MAX);
    when(highLimelight.getDistanceToTarget()).thenReturn(distanceToTarget);

    shootCommand.schedule();
    commandScheduler.run();
    assertFalse(shootCommand.isFinished());
    commandScheduler.run();

    InOrder inOrder = inOrder(shooter, indexer, drivetrain);
    inOrder.verify(shooter).prepareToShoot(Units.metersToInches(distanceToTarget));
    inOrder.verify(drivetrain).arcadeDrive(eq(0.0), gt(0.0), eq(false));
    inOrder.verify(shooter).isReadyToShoot();
    inOrder.verify(shooter).prepareToShoot(Units.metersToInches(distanceToTarget));
    inOrder.verify(drivetrain).arcadeDrive(0.0, -0.0, false);
    inOrder.verify(shooter).isReadyToShoot();
    inOrder.verify(indexer).shoot();
    inOrder.verify(shooter).stopShooter();
    inOrder.verify(indexer).stopIndexer();
    inOrder.verify(drivetrain).stop();

    assertTrue(shootCommand.isFinished());
  }

  
  @Test
  public void testSpinUpAndTargeting() {

    var distanceToTarget = 1d;
    when(highLimelight.getTargetAcquired()).thenReturn(true);
    when(highLimelight.getTargetX()).thenReturn(5d, 0d); // Not at target until second call
    when(shooter.isReadyToShoot()).thenReturn(false, true); // Not ready until second call
    when(highLimelight.getMaxX()).thenReturn(LimeLightConstants.TARGET_X_MAX);
    when(highLimelight.getDistanceToTarget()).thenReturn(distanceToTarget);

    shootCommand.schedule();
    commandScheduler.run();
    assertFalse(shootCommand.isFinished());
    commandScheduler.run();

    InOrder inOrder = inOrder(shooter, indexer, drivetrain);
    inOrder.verify(shooter).prepareToShoot(distanceToTarget);
    inOrder.verify(drivetrain).arcadeDrive(eq(0.0), gt(0.0), eq(false));
    inOrder.verify(shooter).isReadyToShoot();
    inOrder.verify(shooter).prepareToShoot(distanceToTarget);
    inOrder.verify(drivetrain).arcadeDrive(0.0, -0.0, false);
    inOrder.verify(shooter).isReadyToShoot();
    inOrder.verify(indexer).shoot();
    inOrder.verify(shooter).stopShooter();
    inOrder.verify(indexer).stopIndexer();
    inOrder.verify(drivetrain).stop();

    assertTrue(shootCommand.isFinished());
  }

  @Test
  public void testNoTarget() {

    when(highLimelight.getTargetAcquired()).thenReturn(false);
    when(lowLimelight.getTargetAcquired()).thenReturn(false);
    
    shootCommand.schedule();
    commandScheduler.run();

    verify(shooter).stopShooter();
    verify(indexer).stopIndexer();
    verify(drivetrain).stop();
    verifyNoMoreInteractions(shooter, drivetrain, indexer);

    assertTrue(shootCommand.isFinished());
  }


}