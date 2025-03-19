package commands;

public class CommandScheduler {
    void parallelAction(CommandCore... commands){
        for (CommandCore commandCore : commands) commandCore.init();
        boolean flag = true;
        while(flag){
            flag = false;
            for (CommandCore command : commands) {
                if (command.isFinished()) continue;
                flag = true;
                command.loop();
            }
        }
    }

    void sequentialAction(CommandCore... commands){
        for (CommandCore command : commands){
            command.init();
            while(!command.isFinished()) command.loop();
        }
    }
}
