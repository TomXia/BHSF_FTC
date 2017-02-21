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
    public collector(HardwarePushbot rob, Gamepad gp){
        robot=rob;
        gp1=gp;
    }

    @Override
    public void run() {
        robot.collector.setPower(-1.0);
        while(gp1.b == true)
        {
        }
        while(gp1.b == false)
        {
        }
        robot.collector.setPower(0.0);
        while(gp1.b == true)
        {
        }
    }
}
