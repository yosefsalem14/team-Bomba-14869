package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ThreadPool;
@Disabled
public class hardware implements Runnable{
    private Robot robot;
    private HardwareMap hw;
    public hardware(Robot robot, HardwareMap hw) {
        this.robot = robot;
        this.hw = hw;
    }

    @Override
    public void run(){

            robot.init(hw);
        try{
            Thread.sleep(1000);
        }catch(InterruptedException e){
            Thread.currentThread().interrupt();
        }
    }
}