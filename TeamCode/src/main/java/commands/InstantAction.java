package commands;

public class InstantAction extends CommandCore{
    private InstantFunction run;
    public InstantAction(InstantFunction run){
        this.run = run;
    }

    @Override
    public void initialize() {
        run.run();
    }

    @Override
    public void loop() {

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
