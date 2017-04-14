package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Administrator on 2017/2/26.
 */

public class musicPlayer extends Thread {
    public MediaPlayer mp = null;
    private Gamepad gp2;
    private HardwareMap hw;
    public boolean isPlaying = false;
    public boolean canRun = true;
    private MediaPlayer.OnCompletionListener lis = null;


    musicPlayer(HardwareMap hardwaremap, Gamepad gamepad){
        lis = new MediaPlayer.OnCompletionListener() {
            @Override
            public void onCompletion(MediaPlayer mp) {
                mp.release();
                mp = null;
                swi();
            }
        };
        hw = hardwaremap;
        gp2 = gamepad;
        mp = MediaPlayer.create(hw.appContext,R.raw.pump_it);
        mp.setOnCompletionListener(lis);
        //mp.start();
    }
    public void swi(){
        mp = MediaPlayer.create(hw.appContext, R.raw.pump_it);
        mp.setOnCompletionListener(lis);
        mp.start();
    }
    @Override
    public void run(){
            while(gp2.left_stick_button){};
            if(isPlaying) {
                mp.pause();
                isPlaying = false;
            }else{
                mp.start();
                isPlaying=true;
            }
        }

}
