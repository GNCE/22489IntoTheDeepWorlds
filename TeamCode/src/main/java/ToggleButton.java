public class ToggleButton {
    private boolean prev = false, stored = false;
    public void input(boolean cur){
        if(!cur) return;
        if(cur == prev) return;
        stored = !stored;
        prev = cur;
    }
    public boolean getVal(){
        return stored;
    }
}
