package org.firstinspires.ftc.teamcode;
import java.lang.Thread;
import org.firstinspires.ftc.teamcode.HardwarePushbot;
/**
 * Created by tomxia on 2017/2/15.
 */

public class shootThread extends Thread{
    public HardwarePushbot robot;
    private long t;
    public shootThread(HardwarePushbot rob){
        robot=rob;
    }
    @Override
    public void run() {
        robot.miniGun.setPower(0.58);
        for(int i = 0; i < 3; ++i)
        {
            robot.LOP=i;
            t=System.currentTimeMillis();
            while(!robot.isLoaded){
                robot.miniGun.setPower(0.0);
                if(System.currentTimeMillis()-t > 5000) return ;
            }
            robot.miniGun.setPower(0.58);
            while(!robot.TCH.isPressed()){}
            robot.miniGun.setPower(0.0);
            try {
                Thread.sleep(60);
            }catch (InterruptedException e){
                e.printStackTrace();
            }
            robot.miniGun.setPower(0.58);

            /*try {
                robot.miniGun.setPower(0);
                if(i==2) Thread.sleep(200);
            }catch (InterruptedException e) {
                e.printStackTrace();
            }*/
            //robot.miniGun.setPower(0.58);
            while (robot.TCH.isPressed())
            {
            }
            robot.isLoaded=false;
            while(!robot.TCH.isPressed())
            {
            }
        }
        robot.miniGun.setPower(0);
    }

}