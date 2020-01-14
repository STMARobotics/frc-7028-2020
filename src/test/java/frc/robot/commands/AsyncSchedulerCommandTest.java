package frc.robot.commands;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.Timeout;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * AsyncSchedulerCommandTest
 */
public class AsyncSchedulerCommandTest {

  private boolean wasRun;
  private CommandScheduler commandScheduler;

  @Rule
  public Timeout timeoutRule = new Timeout(3, TimeUnit.SECONDS);

  @Before
  public void setUp() {
    commandScheduler = CommandScheduler.getInstance();
    wasRun = false;
  }

  /**
   * Tests scheduling a command that sets `wasRun` to true.
   */
  @Test
  public void testSchedule() throws Exception {
    var lockObject = new CountDownLatch(1);
    var command = new AsyncSchedulerCommand(() -> {
        // Sleep a little and then return a command that sets wasRun to true
        try{
          Thread.sleep(200);
        } catch (Exception e) {
        }
        return new InstantWhenDisabledCommand(() -> wasRun = true);
      })
      .andThen(new InstantWhenDisabledCommand(() -> {
        // Release the lock so the test finishes
        synchronized(lockObject) {
          lockObject.countDown();
        }
      }));

    // Scheduled the group with the AsyncSchedulerCommand, followed by releasing the lock
    commandScheduler.schedule(command);

    // Run the scheduler until the lock is released
    synchronized(lockObject) {
      while (!lockObject.await(20, TimeUnit.MILLISECONDS)) {
        commandScheduler.run();
      }
    }

    assertTrue(wasRun);
  }

  /**
   * Tests when the Supplier returns null, so there is no command the schedule.
   */
  @Test
  public void tesNullResult() throws Exception {
    var lockObject = new CountDownLatch(1);
    var command = new AsyncSchedulerCommand(() -> {
      wasRun = true;
      return null;
    }).andThen(new InstantWhenDisabledCommand(() -> {
        synchronized(lockObject) {
          lockObject.countDown();
        }
      }));

    commandScheduler.schedule(command);

    // Run the scheduler until the lock is released
    synchronized(lockObject) {
      while (!lockObject.await(20, TimeUnit.MILLISECONDS)) {
        commandScheduler.run();
      }
    }

    assertTrue(wasRun);
  }

  /**
   * Tests that the thread gets interrupted when the AsyncSchedulerCommand gets interrupted
   */
  @Test
  public void testInterrupt() throws Exception {
    var command = new AsyncSchedulerCommand(() -> {
        try{
          Thread.sleep(2000);
        } catch (Exception e) {
        }
        return new InstantWhenDisabledCommand(() -> wasRun = true);
      });

    commandScheduler.schedule(command);

    command.end(true);

    assertFalse(wasRun);
  }

}