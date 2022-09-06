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

package multiteams;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import TrcCommonLib.trclib.TrcOpenCVDetector;
import TrcCommonLib.trclib.TrcRevBlinkin;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcTensorFlow;
import TrcFtcLib.ftclib.FtcVuforia;

/**
 * This class implements Vuforia/TensorFlow/Grip/Eocv Vision for the game season. It creates and initializes all the
 * vision target info as well as providing info for the robot, camera and the field. It also provides methods to get
 * the location of the robot and detected targets.
 */
public class Vision
{
    public static final String OPENCV_NATIVE_LIBRARY_NAME = "EasyOpenCV";
    public static final String IMAGE1_NAME = "Image1";
    public static final String IMAGE2_NAME = "Image2";
    public static final String IMAGE3_NAME = "Image3";
    public static final String IMAGE4_NAME = "Image4";
    public static final String LABEL_TARGET = "Target";
    public static final String GOT_TARGET = "GotTarget";
    public static final String SAW_TARGET = "SawTarget";

    private final TrcRevBlinkin.Pattern[] ledPatternPriorities = {
        new TrcRevBlinkin.Pattern(SAW_TARGET, TrcRevBlinkin.RevLedPattern.SolidViolet),
        new TrcRevBlinkin.Pattern(GOT_TARGET, TrcRevBlinkin.RevLedPattern.SolidAqua)
    };

    private final Robot robot;
    public VuforiaVision vuforiaVision;
    public TensorFlowVision tensorFlowVision;
    public EocvVision eocvVision;

    /**
     * Constructor: Create an instance of the object. Vision is required by both Vuforia and TensorFlow and must be
     * instantiated if either is used. However, to use either Vuforia or TensorFlow, one must explicitly initialize
     * them by calling the initVuforia or initTensorFlow methods respectively.
     *
     * @param robot specifies the robot object.
     */
    public Vision(Robot robot)
    {
        FtcOpMode opMode = FtcOpMode.getInstance();
        int cameraViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        this.robot = robot;
        if (RobotParams.Preferences.useEasyOpenCV)
        {
            OpenCvCamera webcam =
                OpenCvCameraFactory.getInstance().createWebcam(
                    opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_WEBCAM), cameraViewId);
            eocvVision = new EocvVision(
                "EocvVision", RobotParams.CAMERA_IMAGE_WIDTH, RobotParams.CAMERA_IMAGE_HEIGHT,
                RobotParams.cameraRect, RobotParams.worldRect, webcam, OpenCvCameraRotation.UPRIGHT,
                RobotParams.Preferences.showEasyOpenCvView, null);
        }
        else
        {
            final String VUFORIA_LICENSE_KEY =
                "ARbBwjf/////AAABmZijKPKUWEY+uNSzCuTOUFgm7Gr5irDO55gtIOjsOXmhLzLEILJp45qdPrwMfoBV2Yh7F+Wh8iEjnSA" +
                "NnnRKiJNHy1T9Pr2uufETE40YJth10Twv0sTNSEqxDPhg2t4PJXwRImMaEsTE53fmcm08jT9qMso2+1h9eNk2b4x6DVKgBt" +
                "Tv5wocDs949Gkh6lRt5rAxATYYO9esmyKyfyzfFLMMpfq7/uvQQrSibNBqa13hJRmmHoM2v0Gfk8TCTTfP044/XsOm54u8k" +
                "dv0HfeMBC91uQ/NvWHVV5XCh8pZAzmL5sry1YwG8FSRNVlSAZ1zN/m6jAe98q6IxpwQxP0da/TpJoqDI7x4RGjOs1Areunf";
            //
            // If no camera view ID, do not activate camera monitor view to save power.
            //
            VuforiaLocalizer.Parameters vuforiaParams =
                RobotParams.Preferences.showVuforiaView?
                    new VuforiaLocalizer.Parameters(cameraViewId): new VuforiaLocalizer.Parameters();

            vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
            vuforiaParams.cameraName = opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_WEBCAM);
            vuforiaParams.useExtendedTracking = false;
            vuforiaParams.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
            FtcVuforia vuforia = new FtcVuforia(vuforiaParams);

            vuforiaVision = RobotParams.Preferences.useVuforia? new VuforiaVision(vuforia, robot.blinkin): null;
            tensorFlowVision = RobotParams.Preferences.useTensorFlow? new TensorFlowVision(vuforia, null): null;
        }
    }   //Vision

    /**
     * This method sets up the Blinkin with a priority pattern list and a pattern name map.
     */
    public void setupBlinkin()
    {
        robot.blinkin.setPatternPriorities(ledPatternPriorities);
    }   //setupBlinkin

    /**
     * This method shuts down TensorFlow.
     */
    public void tensorFlowShutdown()
    {
        if (tensorFlowVision != null)
        {
            tensorFlowVision.shutdown();
            tensorFlowVision = null;
        }
    }   //tensorFlowShutdown

    /**
     * This method returns an array of the detected targets info.
     *
     * @param label specifies the target label, only valid for TensorFlowVision, null for others.
     * @return an array of detected targets info.
     */
    public TrcVisionTargetInfo<?>[] getDetectedTargetsInfo(String label)
    {
        TrcVisionTargetInfo<?>[] targets = null;

        if (tensorFlowVision != null)
        {
            targets = tensorFlowVision.getDetectedTargetsInfo(label, null, this::compareConfidence, false);
        }
        else if (eocvVision != null)
        {
            targets = eocvVision.getDetectedTargetsInfo(null, this::compareObjectSize);
        }

        return targets;
    }   //getDetectedTargetsInfo

    /**
     * This method returns the best detected target info.
     *
     * @param label specifies the target label, only valid for TensorFlowVision, null for others.
     * @return best detected target info.
     */
    public TrcVisionTargetInfo<?> getBestDetectedTargetInfo(String label)
    {
        TrcVisionTargetInfo<?>[] targets = getDetectedTargetsInfo(label);

        return targets != null? targets[0]: null;
    }   //getDetectedTargetsInfo

    /**
     * This method returns info of the closest detected target to image center.
     *
     * @param label specifies the target label, only valid for TensorFlowVision, null for others.
     * @return closest detected target info.
     */
    public TrcVisionTargetInfo<?> getClosestTargetInfo(String label)
    {
        TrcVisionTargetInfo<?>[] targets = null;

        if (tensorFlowVision != null)
        {
            targets = tensorFlowVision.getDetectedTargetsInfo(label, null, this::compareDistanceFromCamera, false);
        }
        else if (eocvVision != null)
        {
            targets = eocvVision.getDetectedTargetsInfo(null, this::compareDistanceFromCamera);
        }

        return targets != null? targets[0]: null;
    }   //getClosestTargetInfo

    /**
     * This method is called by the Arrays.sort to sort the target object by decreasing confidence.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has higher confidence than b, 0 if a and b have equal confidence, positive value
     *         if a has lower confidence than b.
     */
    private int compareConfidence(
        TrcVisionTargetInfo<FtcTensorFlow.DetectedObject> a, TrcVisionTargetInfo<FtcTensorFlow.DetectedObject> b)
    {
        return (int)((b.detectedObj.confidence - a.detectedObj.confidence)*100);
    }   //compareConfidence

    /**
     * This method is called by the Arrays.sort to sort the target object by increasing object distance.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has smaller distance to image center than b, 0 if a and b have equal distance to
     * image center, positive value if a has larger distance to image center than b.
     */
    private int compareDistanceFromCamera(TrcVisionTargetInfo<?> a, TrcVisionTargetInfo<?> b)
    {
        return (int)((Math.abs(a.distanceFromCamera.y) - Math.abs(b.distanceFromCamera.y))*1000);
    }   //compareDistanceFromCamera

    /**
     * This method is called by the Arrays.sort to sort the target object by increasing camera angle.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has smaller camera angle than b, 0 if a and b have equal camera angle, positive
     *         value if a has larger camera angle than b.
     */
    private int compareCameraAngle(TrcVisionTargetInfo<?> a, TrcVisionTargetInfo<?> b)
    {
        return (int)((Math.abs(a.horizontalAngle) - Math.abs(b.horizontalAngle))*1000);
    }   //compareCameraAngle

    /**
     * This method is called by the Arrays.sort to sort the target object by decreasing object size.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has smaller area than b, 0 if a and b have equal area, positive value if a has
     *         larger area than b.
     */
    private int compareObjectSize(
        TrcVisionTargetInfo<TrcOpenCVDetector.DetectedObject> a,
        TrcVisionTargetInfo<TrcOpenCVDetector.DetectedObject> b)
    {
        return a.detectedObj.rect.width * a.detectedObj.rect.height -
               b.detectedObj.rect.width * b.detectedObj.rect.height;
    }   //compareObjectSize

}   //class Vision
