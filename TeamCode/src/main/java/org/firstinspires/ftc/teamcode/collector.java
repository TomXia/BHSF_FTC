package org.firstinspires.ftc.teamcode;
import java.lang.Thread;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.HardwarePushbot;

/**
 * Created by tomxia on 2017/2/15.
 */

public class collector extends Thread {
    public HardwarePushbot robot;
    public Gamepad gp1;
    boolean mode2;
    public collector(HardwarePushbot rob, Gamepad gp,boolean mode){
        robot=rob;
        gp1=gp;
        mode2 = mode;
    }

    @Override
    public void run() {
        robot.collector.setPower(mode2 ? 1:-1.0);
        robot.wipeYellow.setPower(mode2 ? 1:-1.0);
        while(mode2 ? gp1.a : gp1.y)
        {
        }
        while(!mode2 && !gp1.y)
        {
        }
        robot.collector.setPower(0.0);
        robot.wipeYellow.setPower(0.0);
        while(mode2 ? false : gp1.y)
        {
        }
    }
}
