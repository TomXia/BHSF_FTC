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
        double delta=0.3*(lenth-len)*(lenth-len)*(lenth-len)+0.05*(lenth-len);
        if(lenth==0||lenth>=150){

        }
        else{
            robot.pushOnebyOne(0.4+0.002*delta,0.4-0.002*delta);
        }


    }
    public void test()
    {
        CaptureResult r;
        CaptureRequest q;
    }
}
