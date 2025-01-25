public class ToggleButton {
    private boolean prev = false, stored;

    public ToggleButton(boolean defaultValue){
        this.stored = defaultValue;
    }
    public boolean input(boolean cur){
        boolean diff = cur && !prev;
        if(diff) stored = !stored;
        prev = cur;
        return diff;
    }
    public boolean getVal(){
        return stored;
    }

    public void setVal(boolean newVal){
        stored = newVal;
    }
}
