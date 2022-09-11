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

package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Calibrate Steering Servos", group="Test")
public class CalibrateSteering extends LinearOpMode {

   // Declare OpMode members.
   private ElapsedTime runtime = new ElapsedTime();
   private Servo servo1, servo2, servo3, servo4;
   private double servoPos = 0.0;
   private boolean buttonWasPressed = false;

   private void setServoPosition(double pos)
   {
      servo1.setPosition(servoPos);
      servo2.setPosition(servoPos);
      servo3.setPosition(servoPos);
      servo4.setPosition(servoPos);
   }

   @Override
   public void runOpMode() {
      telemetry.addData("Status", "Initialized");
      telemetry.update();

      // Initialize the hardware variables. Note that the strings used here as parameters
      // to 'get' must correspond to the names assigned during the robot configuration
      // step (using the FTC Robot Controller app on the phone).
      servo1 = hardwareMap.get(Servo.class, "servo1");
      servo2 = hardwareMap.get(Servo.class, "servo2");
      servo3 = hardwareMap.get(Servo.class, "servo3");
      servo4 = hardwareMap.get(Servo.class, "servo4");
      setServoPosition(servoPos);

      // Wait for the game to start (driver presses PLAY)
      waitForStart();
      runtime.reset();

      // run until the end of the match (driver presses STOP)
      while (opModeIsActive()) {
         boolean buttonIsPressed = gamepad1.a;
         boolean buttonHasEvent = false;

         if (!buttonWasPressed && buttonIsPressed)
         {
            // Button is pressed.
            buttonHasEvent = true;
         }
         else if (buttonWasPressed && !buttonIsPressed)
         {
            // Button is released.
            buttonHasEvent = true;
         }

         if (buttonHasEvent)
         {
            servoPos += 0.5;
            if (servoPos > 1.0)
            {
               servoPos = 0.0;
            }

            setServoPosition(servoPos);
         }

         buttonWasPressed = buttonIsPressed;

         // Show the elapsed game time and wheel power.
         telemetry.addData("Status", "Run Time: " + runtime.toString());
         telemetry.addData("Servos", "pos=%.1f", servoPos);
         telemetry.update();
      }
   }
}
