package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by Administrator on 2017/2/26.
 */

public class ballSeeker {
    private TouchSensor  tch = null;
    final static double threshold = 0.28;
    ballSeeker(TouchSensor touchSensor){
        tch = touchSensor;
    }
   public boolean getResault(){
       return  (tch.getValue()==1.0);
   }
}
