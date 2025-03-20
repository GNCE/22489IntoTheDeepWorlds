package commands;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class CommandCore {
    private ElapsedTime elapsedTime = new ElapsedTime();

    public CommandCore(){

    }

    public void init(){
        elapsedTime.reset();
        elapsedTime.startTime();
        initialize();
    }
    public abstract void initialize();
    public abstract void loop();
    public boolean isFinished(){ return false; }
}