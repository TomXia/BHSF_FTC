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
        if(delta>0.2){
            delta=0.2;
        }
        else if(delta<-0.2){
            delta=-0.2;
        }
        x=0.4+delta;
        y=0.4-delta;
        if(lenth==0||lenth>=150||lenth==24){
            robot.pushOnebyOne(0,0);
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
