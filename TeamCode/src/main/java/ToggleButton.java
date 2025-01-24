public class ToggleButton {
    private boolean prev = false, stored;

    public ToggleButton(boolean defaultValue){
        this.stored = defaultValue;
    }
    public void input(boolean cur){
        if(cur && !prev) stored = !stored;
        prev = cur;
    }
    public boolean getVal(){
        return stored;
    }
}
