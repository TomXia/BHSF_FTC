package org.firstinspires.ftc.teamcode;

import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;

/**
 * Created by Administrator on 2017/3/14.
 */

public class subAuto {
    HardwarePushbot robot;
    public subAuto(HardwarePushbot r)
    {
        this.robot = r;
    }

    public void ultrasonicgo()
    {
        double len=16.0;
        double lenth=(robot.uls.getUltrasonicLevel());
        double delta=0.02*(lenth-len);
        double x,y;
        if((8*delta-delta*delta)<0){
            delta=(Math.sqrt(Math.abs(8*delta-delta*delta)))/16*(-1);
        }
        else{
            delta=(Math.sqrt(8*delta-delta*delta))/16;
        }

        if(delta>0.25){
            delta=0.25;
        }
        else if(delta<-0.25){
            delta=-0.25;
        }
        x=0.6+delta;
        y=0.6-delta;
        if(lenth==0||lenth>=150){
            robot.pushOnebyOne(0,0);
        }
        else if(lenth==24){
            robot.pushOnebyOne(0.6,0.6);
        }
        else{
            robot.pushOnebyOne(x,y);
        }


    }
    public void test()
    {
        CaptureResult r;
        CaptureRequest q;
    }
}
