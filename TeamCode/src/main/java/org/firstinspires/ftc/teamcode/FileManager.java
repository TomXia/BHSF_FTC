package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.util.concurrent.BlockingQueue;
import java.io.IOException;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Administrator on 2017/2/26.
 */

public class FileManager extends Thread {
    public boolean isWrite;
    public CommandInfo cm;
    public File fs;
    public FileOutputStream ofs;
    public FileInputStream ifs;
    public BlockingQueue<CommandInfo> que;
    public DataInputStream input;
    public DataOutputStream output;
    public Gamepad gp;
    private long time;

    public FileManager(boolean isW,Gamepad gamepad, BlockingQueue<CommandInfo> q)
    {
        isWrite=isW;
        que=q;
        fs = new File(Environment.getExternalStorageDirectory().getPath()+File.separator+"BHSF_FTC"+File.separator+"record.rc");
        cm = new CommandInfo();
//        gp = gamepad;
        if (!fs.getParentFile().exists()) {
            if (!fs.getParentFile().mkdirs()) {
            }
        }
        if(!fs.exists()) {
            try {
                fs.createNewFile();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        if(isW) {
            try {
                ofs = new FileOutputStream(fs);
            } catch (IOException e) {
                e.printStackTrace();
            }
            output = new DataOutputStream(ofs);
        }
        else {
            try {
                ifs = new FileInputStream(fs);
            } catch (IOException e) {
                e.printStackTrace();
            }
            input = new DataInputStream(ifs);
        }
    }
    @Override
    public void run() {
        time = System.currentTimeMillis();
        while(!gp.left_bumper) {
            if (isWrite) {
                try {
                    cm = que.take();
                }catch(InterruptedException e) {
                    e.printStackTrace();
                }
                try {
                    output.writeLong(cm.time);
                    output.writeBytes(cm.cmd);
                    output.writeDouble(cm.val);
                }catch (IOException e){
                    e.printStackTrace();
                }
                } else {
                try {
                    cm.time=input.readLong();
                    cm.cmd=input.readUTF();
                    cm.val=input.readDouble();
                }catch (IOException e){
                    e.printStackTrace();
                }
                try {
                    que.put(cm);
                }catch(InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }
}
