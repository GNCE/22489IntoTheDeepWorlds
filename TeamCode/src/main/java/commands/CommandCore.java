package commands;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class CommandCore {
    ElapsedTime elapsedTime = new ElapsedTime();

    public void init(){
        elapsedTime.reset();
        elapsedTime.startTime();
        initialize();
    }
    abstract public void initialize();
    abstract public void loop();
    public boolean isFinished(){ return false; }
}