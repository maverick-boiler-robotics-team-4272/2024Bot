package frc.robot.commands;

import java.util.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.paths.TrajectoryContainer.Path;

public class PathFollowWithEvents extends Command {
    private Command pathFollowCommand;
    private List<Pair<Double, Command>> unstartedCommands;
    private List<Command> runningCommands;
    private Timer timer;

    public PathFollowWithEvents(Command pathFollowCommand, Path path) {
        m_requirements.addAll(pathFollowCommand.getRequirements());

        this.unstartedCommands = new ArrayList<>(path.trajectory.getEventCommands());
        this.unstartedCommands.sort((a, b) -> {
            return Double.compare(a.getFirst(), b.getFirst());
        });


        for(Pair<Double, Command> p : unstartedCommands) {
            if(!Collections.disjoint(pathFollowCommand.getRequirements(), p.getSecond().getRequirements()))
                throw new IllegalArgumentException("You cannot have a marker use a command that requires subsystems required by the path follow command");
            m_requirements.addAll(p.getSecond().getRequirements());
        }

        this.pathFollowCommand = pathFollowCommand;

        this.runningCommands = new ArrayList<>();
        this.timer = new Timer();
    }

    @Override
    public void initialize() {
        pathFollowCommand.initialize();

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        pathFollowCommand.execute();

        double time = timer.get();
        
        for(int i = 0; i < unstartedCommands.size(); i++) {
            Pair<Double, Command> event = unstartedCommands.get(i);
            if(time > event.getFirst()) {
                Command command = event.getSecond();

                for(int j = 0; j < runningCommands.size(); j++) {
                    Command checkCommand = runningCommands.get(j);
                    if(!Collections.disjoint(command.getRequirements(), checkCommand.getRequirements())) {
                        checkCommand.end(true);
                        runningCommands.remove(j);
                        j--;
                    }
                }

                runningCommands.add(command);
                command.initialize();

                unstartedCommands.remove(i);
                i--;
            } else {
                break;
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

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        pathFollowCommand.end(interrupted);

        for(Command command : runningCommands) {
            command.end(true);
        }
    }
}
