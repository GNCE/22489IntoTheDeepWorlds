package config.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import config.core.utils.SubsystemCore;


public class LooptimeTracker extends SubsystemCore {
    private ElapsedTime timer;
    @Override
    public void init(){
        timer = new ElapsedTime();
        timer.startTime();
    }

    @Override
    public void periodic(){
        tel.addData("Loop Time", timer.time());
        timer.reset();
    }
}