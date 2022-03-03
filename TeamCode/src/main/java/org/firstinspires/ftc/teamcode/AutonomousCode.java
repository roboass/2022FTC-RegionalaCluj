/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutonomousCode", group="Linear Opmode")
//@Disabled
public class AutonomousCode extends UsefulFunctions {
    private ElapsedTime runtime = new ElapsedTime();
    String position;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Initialise();
        InitialiseVision();

        waitForStart();
        runtime.reset();

        sleep(500);
        position = pipeline.getPosition();
        telemetry.addData("pos", position);
        telemetry.update();
        StopVision();

//        trafaletServoDreapta.setPosition(90/180);
//        trafaletServoStanga.setPosition(90/180);
        rampaServoDreapta.setPosition(0.5);
        rampaServoStanga.setPosition(0.5);

        sleep(200);

        if(position == "FIRST") {
            addToRampaAngle(-rampaAngle + unghiNivelJos);
        } else if(position == "SECOND") {
            addToRampaAngle(-rampaAngle + unghiNivelMij);

        } else if(position == "THIRD") {
            addToRampaAngle(-rampaAngle + unghiNivelSus);
        }
        sleep(200);

        AutonomousMove(0, in_to_mm(24));
        sleep(200);
        AutonomousMove(-in_to_mm(13), 0);
        sleep(200);

        motorRampaOnOff();
        sleep(7500);
        motorRampaOnOff();

        telemetry.addData("angle", gyro.getAngularOrientation().firstAngle);
        telemetry.update();
        sleep(1000);

        AutonomousRotate(105);
        telemetry.addData("angle", gyro.getAngularOrientation().firstAngle);
        telemetry.update();

        sleep(500);
        AutonomousMove(-in_to_mm(36), 0);
        sleep(500);

        motorRampaOnOff();
        sleep(7500);
        motorRampaOnOff();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Position", "Position: " + position);
            telemetry.update();
        }

    }
}