package org.firstinspires.ftc.teamcode.utilities;

public class DetectedObject {
    private int pos;
    private int ID;
    private double angle;
    public DetectedObject(int pos,double angle){
        this.pos = pos;
        this.angle = angle;
        this.ID = -1;
    }

    public int getPos(){
        return this.pos;
    }
    public void setID(int num){
        this.ID = num;
    }
    public int getID(){
        return this.ID;
    }
    public double getAngle(){
        return this.angle;
    }


}
