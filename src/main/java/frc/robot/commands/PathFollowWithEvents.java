package frc.robot.commands;

import java.util.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.paths.TrajectoryContainer.Path;

public class PathFollowWithEvents extends Command {
    private Command pathFollowCommand;
    private List<Pair<Double, Command>> unstartedCommands;
    private List<Pair<Double, Command>> pauseTimes;
    private List<Command> runningCommands;
    private Timer timer;
    private boolean paused;

    public PathFollowWithEvents(Command pathFollowCommand, Path path) {
        m_requirements.addAll(pathFollowCommand.getRequirements());

        this.unstartedCommands = new ArrayList<>(path.trajectory.getEventCommands());
        this.unstartedCommands.sort((a, b) -> {
            return Double.compare(a.getFirst(), b.getFirst());
        });

        this.pathFollowCommand = pathFollowCommand;

        this.runningCommands = new ArrayList<>();
        this.pauseTimes = new ArrayList<>();
        this.timer = new Timer();
        this.paused = false;
    }

    @Override
    public void initialize() {
        pathFollowCommand.initialize();

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if(!paused)
            pathFollowCommand.execute();

        double time = timer.get();
        double checkTime = 0;
        Pair<Double, Command> event = null;

        if(pauseTimes.size() > 0) {
            event = pauseTimes.get(0);
            checkTime = event.getFirst();
            
            while(time > checkTime) {
                Command command = event.getSecond();
                pause();

                interruptCommands(command);

                runningCommands.add(command);
                command.initialize();
                
                pauseTimes.remove(0);

                if(pauseTimes.size() == 0)
                    break;
                
                event = pauseTimes.get(0);
                checkTime = event.getFirst();
            }
        }

        if(unstartedCommands.size() > 0) {
            event = unstartedCommands.get(0);
            checkTime = event.getFirst();

            while(time > checkTime) {
                Command command = event.getSecond();

                if(!paused && !Collections.disjoint(command.getRequirements(), pathFollowCommand.getRequirements())) {
                    System.out.println("Cannot run a command that requires the path following command when running the pathfollow");
                    continue;
                }

                interruptCommands(command);

                runningCommands.add(command);
                command.initialize();

                unstartedCommands.remove(0);

                if(unstartedCommands.size() == 0)
                    break;
                event = unstartedCommands.get(0);
                checkTime = event.getFirst();
            }
        }

        for(int i = 0; i < runningCommands.size(); i++) {
            Command c = runningCommands.get(i);
            c.execute();

            if(c.isFinished()) {
                c.end(false);
                runningCommands.remove(i);
                i--;
            }
        }
    }

    private void interruptCommands(Command command) {
        for(int i = 0; i < runningCommands.size(); i++) {
            Command checkCommand = runningCommands.get(i);
            if(!Collections.disjoint(command.getRequirements(), checkCommand.getRequirements())) {
                checkCommand.end(true);
                runningCommands.remove(i);
                i--;
            }
        }
    }

    public void pause() {
        timer.stop();
        paused = true;
    }

    public void unpause() {
        timer.start();
        paused = false;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        pathFollowCommand.end(interrupted);

        for(Command command : runningCommands) {
            command.end(true);
        }
    }

    public void addPauseTime(double time, Command command) {
        pauseTimes.add(new Pair<>(time, command));
    }
}
