package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

/**
 * Created by Administrator on 2017/2/26.
 */

public class ballSeeker {
    private DigitalChannel dc;
    ballSeeker(DigitalChannel d){
        dc = d;
        dc.setMode(DigitalChannelController.Mode.INPUT);
    }
   public double getResault(){
       if(dc.getState())
           return 4.0;
       else
           return 20.0;
   }
}
