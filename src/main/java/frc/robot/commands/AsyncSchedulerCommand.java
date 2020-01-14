package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Calls a Supplier on another thread to create a command, and then schedules it on the standard command thread.
 * <p>This command is useful for scheduling a command that requires a computation to be run during it's creation that
 * may take too long to run on the standard command loop.</p>
 * <p>You should prefer creating such commands in robotInit() instead of doing it later on the fly, whenever possible.
 * </p>
 */
public class AsyncSchedulerCommand extends CommandBase {

  private Thread supplierThread;
  private Command suppliedCommand;
  
  public AsyncSchedulerCommand(Supplier<Command> commandSupplier) {
    supplierThread = new Thread(() -> suppliedCommand = commandSupplier.get(), "AsyncSchedulerCommand Thread");
  }

  @Override
  public void initialize() {
    supplierThread.start();
  }

  @Override
  public boolean isFinished() {
    return !supplierThread.isAlive();
  }

  @Override
  public void end(boolean interrupted) {
    if (null == suppliedCommand) {
      System.out.println("AsyncSchedulerCommand did not have anything to schedule");
    } else {
      suppliedCommand.schedule();
    }
    if (interrupted) {
      System.out.println("AsyncSchedulerCommand interrupted");
      supplierThread.interrupt();
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}