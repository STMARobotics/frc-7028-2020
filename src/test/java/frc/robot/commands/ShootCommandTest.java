package frc.robot.commands;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.AdditionalMatchers.gt;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.inOrder;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.times;
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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ShooterConstants;
import frc.robot.networktables.DoubleEntryValue;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * ShootCommandTest
 */
@RunWith(MockitoJUnitRunner.class)
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
    shootCommand = spy(new ShootCommand(1, shooter, indexer, highLimelight, lowLimelight, drivetrain));
    when(shootCommand.runsWhenDisabled()).thenReturn(true);
  }

  @Test
  public void testImmediateShoot() {

    var distanceToTarget = 1d;
    when(highLimelight.getRawTargetValid()).thenReturn(new DoubleEntryValue(1.0));
    when(highLimelight.getTargetX()).thenReturn(0d);
    when(shooter.isReadyToShoot()).thenReturn(true);
    when(highLimelight.getDistanceToTarget()).thenReturn(distanceToTarget);
    when(indexer.isFull()).thenReturn(true, false);

    shootCommand.schedule();
    commandScheduler.run();
    Timer timer = new Timer();
    timer.start();
    do {
      Timer.delay(.02);
      commandScheduler.run();
    } while(!timer.hasPeriodPassed(ShooterConstants.SHOOT_TIME));
    commandScheduler.run();

    InOrder inOrder = inOrder(shooter, indexer, drivetrain);
    inOrder.verify(shooter).prepareToShoot(Units.metersToInches(distanceToTarget));
    inOrder.verify(drivetrain).arcadeDrive(0.0, -0.0, false);
    inOrder.verify(shooter).isReadyToShoot();
    inOrder.verify(indexer).shoot();
    inOrder.verify(shooter).stopShooter();
    inOrder.verify(indexer).stopIndexer();
    inOrder.verify(drivetrain).stop();
  }

  @Test
  public void testLowLimelightImmediateShoot() {

    var distanceToTarget = 1d;
    when(highLimelight.getRawTargetValid()).thenReturn(new DoubleEntryValue(0.0));
    when(lowLimelight.getRawTargetValid()).thenReturn(new DoubleEntryValue(1.0));
    when(lowLimelight.getTargetX()).thenReturn(0d);
    when(shooter.isReadyToShoot()).thenReturn(true);
    when(lowLimelight.getDistanceToTarget()).thenReturn(distanceToTarget);
    when(indexer.isFull()).thenReturn(true, false);

    shootCommand.schedule();
    commandScheduler.run();
    Timer timer = new Timer();
    timer.start();
    do {
      Timer.delay(.02);
      commandScheduler.run();
    } while(!timer.hasPeriodPassed(ShooterConstants.SHOOT_TIME));
    commandScheduler.run();

    InOrder inOrder = inOrder(shooter, indexer, drivetrain);
    inOrder.verify(shooter).prepareToShoot(Units.metersToInches(distanceToTarget));
    inOrder.verify(drivetrain).arcadeDrive(0.0, -0.0, false);
    inOrder.verify(shooter).isReadyToShoot();
    inOrder.verify(indexer).shoot();
    inOrder.verify(shooter).stopShooter();
    inOrder.verify(indexer).stopIndexer();
    inOrder.verify(drivetrain).stop();
  }

  @Test
  public void testSpinUp() {

    var distanceToTarget = 1d;
    when(highLimelight.getRawTargetValid()).thenReturn(new DoubleEntryValue(1.0));
    when(highLimelight.getTargetX()).thenReturn(0d);
    when(shooter.isReadyToShoot()).thenReturn(false, true); // Not at target until second call
    when(highLimelight.getDistanceToTarget()).thenReturn(distanceToTarget);
    when(indexer.isFull()).thenReturn(true, false, true);

    shootCommand.schedule();
    commandScheduler.run();
    assertFalse(shootCommand.isFinished());
    commandScheduler.run();
    Timer timer = new Timer();
    timer.start();
    do {
      Timer.delay(.02);
      commandScheduler.run();
    } while(!timer.hasPeriodPassed(ShooterConstants.SHOOT_TIME));
    commandScheduler.run();

    InOrder inOrder = inOrder(shooter, indexer, drivetrain);
    inOrder.verify(shooter).prepareToShoot(Units.metersToInches(distanceToTarget));
    inOrder.verify(shooter).isReadyToShoot();
    inOrder.verify(shooter).prepareToShoot(Units.metersToInches(distanceToTarget));
    inOrder.verify(shooter).isReadyToShoot();
    inOrder.verify(indexer).shoot();
    inOrder.verify(shooter).stopShooter();
    inOrder.verify(indexer).stopIndexer();
    inOrder.verify(drivetrain).stop();
  }

  @Test
  public void testTargeting() {

    var distanceToTarget = 1d;
    when(highLimelight.getRawTargetValid()).thenReturn(new DoubleEntryValue(1.0));
    when(highLimelight.getTargetX()).thenReturn(5d, 0d); // Not at target until second call
    when(shooter.isReadyToShoot()).thenReturn(true);
    when(highLimelight.getDistanceToTarget()).thenReturn(distanceToTarget);
    when(indexer.isFull()).thenReturn(true, false);

    shootCommand.schedule();
    commandScheduler.run();
    assertFalse(shootCommand.isFinished());
    commandScheduler.run();
    commandScheduler.run();
    Timer timer = new Timer();
    timer.start();
    do {
      commandScheduler.run();
      Timer.delay(.02);
    } while(!timer.hasPeriodPassed(ShooterConstants.SHOOT_TIME));
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
  }

  
  @Test
  public void testSpinUpAndTargeting() {

    var distanceToTarget = 1d;
    when(highLimelight.getRawTargetValid()).thenReturn(new DoubleEntryValue(1.0));
    when(highLimelight.getTargetX()).thenReturn(5d, 0d); // Not at target until second call
    when(shooter.isReadyToShoot()).thenReturn(false, true); // Not ready until second call
    when(highLimelight.getDistanceToTarget()).thenReturn(distanceToTarget);
    when(indexer.isFull()).thenReturn(true, false, true);

    shootCommand.schedule();
    commandScheduler.run();
    assertFalse(shootCommand.isFinished());
    commandScheduler.run();
    Timer timer = new Timer();
    timer.start();
    do {
      commandScheduler.run();
      Timer.delay(.02);
    } while(!timer.hasPeriodPassed(ShooterConstants.SHOOT_TIME));
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
  }

  @Test
  @Ignore("This test is broken by lost target buffer")
  public void testNoTarget() {

    when(highLimelight.getRawTargetValid()).thenReturn(new DoubleEntryValue(0.0));
    when(lowLimelight.getRawTargetValid()).thenReturn(new DoubleEntryValue(0.0));
    
    shootCommand.schedule();
    commandScheduler.run();

    verify(shooter).stopShooter();
    verify(indexer).stopIndexer();
    verify(drivetrain, times(2)).stop();
    verify(indexer, times(2)).isFull();
    verifyNoMoreInteractions(shooter, drivetrain, indexer);

    assertTrue(shootCommand.isFinished());
  }


}