package org.firstinspires.ftc.teamcode.Auto_;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class Example extends LinearOpMode {
    enum state{
        Wait_For_A,
        Wait_For_B,
        Finished;
    }
    @Override
    public void runOpMode(){
        waitForStart();
        while(opModeIsActive()){
            state state = Example.state.Wait_For_A;
            telemetry.addData("進度：",state);
            switch (state){                          //迴圈
                case Wait_For_A:                         //迴圈中的迴圈
                    telemetry.addLine("To exit this state,Press A");
                    if(gamepad1.a){
                        state= state.Wait_For_B;  //用變數切換迴圈
                    }
                    break;   //跳出
                case Wait_For_B:
                    telemetry.addLine("To exit this state,Press B");
                    if(gamepad1.b){
                        state= state.Finished;
                    }
                    break;
                default:
                    telemetry.addLine("This Autonomous is already finish");
            }
        }
    }
}
