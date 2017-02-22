package org.firstinspires.ftc.teamcode;
import java.lang.Thread;
import org.firstinspires.ftc.teamcode.HardwarePushbot;
/**
 * Created by tomxia on 2017/2/15.
 */

public class shootThread extends Thread{
    public HardwarePushbot robot;
    public shootThread(HardwarePushbot rob){
        robot=rob;
    }
    @Override
    public void run() {
        robot.miniGun.setPower(0.58);
        for(int i = 0; i < 3; ++i)
        {
            while(!robot.TCH.isPressed())
            {
            }
            while (robot.TCH.isPressed())
            {
            }
            robot.isLoaded=false;
            robot.isReadytoLoad=true;
            while(!robot.TCH.isPressed())
            {
            }
        }
        robot.miniGun.setPower(0);
    }

}