/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package multiteams.team6541;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import TrcCommonLib.trclib.TrcRobot;
import multiteams.FtcAuto;

/**
 * This class contains the Autonomous Mode program.
 */
@Autonomous(name="FtcAuto6541", group="FtcAuto")
public class FtcAuto6541 extends FtcAuto
{
    //
    // Implements FtcOpMode abstract method.
    //

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station is pressed.
     */
    @Override
    public void initRobot()
    {
        //
        // Create and initialize team specific robot object.
        //
        robot = new Robot6541(TrcRobot.getRunMode());
        //
        // Do common init.
        //
        super.initRobot();
        //
        // Create team specific autonomous command according to chosen strategy.
        //
        switch (autoChoices.strategy)
        {
            default:
                break;
        }
    }   //initRobot

}   //class FtcAuto6541
