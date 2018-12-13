package org.firstinspires.ftc.teamcode.utilities;

public enum ObjectPositions {

    LEFT,
    CEMTER,
    RIGHT,
    UNKNOWN;
    private int value = 0;

    public int getIntVal(){
        switch (this){
            case LEFT:
                value = 1;
                break;
            case CEMTER:
                value = 2;
                break;
            case RIGHT:
                value = 3;
                break;
            default:
                value = -1;
                break;
        }
        return value;
    }
}
