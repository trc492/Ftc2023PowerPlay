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

package teamcode;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcHomographyMapper;
import TrcCommonLib.trclib.TrcOpenCVDetector;
import TrcCommonLib.trclib.TrcUtil;
import TrcFtcLib.ftclib.FtcEocvDetector;

/**
 * This class implements EOCV Vision that provides the capability to detect color blobs and return their location
 * info.
 */
public class EocvVision extends FtcEocvDetector
{
    private static final double[] redThresholdRedCone = {128.0, 255.0};
    private static final double[] greenThresholdRedCone = {0.0, 100.0};
    private static final double[] blueThresholdRedCone = {0.0, 60.0};

    private static final double[] redThresholdBlueCone = {0.0, 100.0};
    private static final double[] greenThresholdBlueCone = {0.0, 100.0};
    private static final double[] blueThresholdBlueCone = {100.0, 255.0};

    private static final double[] redThresholdYellowPole = {128.0, 255.0};
    private static final double[] greenThresholdYellowPole = {128.0, 255.0};
    private static final double[] blueThresholdYellowPole = {0.0, 60.0};

    public enum ObjectType
    {
        RED_CONE, BLUE_CONE, YELLOW_POLE;

        static ObjectType nextObjectType(ObjectType objType)
        {
            ObjectType nextObjType;

            switch (objType)
            {
                case RED_CONE:
                    nextObjType = BLUE_CONE;
                    break;

                case BLUE_CONE:
                    nextObjType = YELLOW_POLE;
                    break;

                default:
                case YELLOW_POLE:
                    nextObjType = RED_CONE;
                    break;
            }

            return nextObjType;
        }   //nextObjectType

    }   //enum ObjectType

    private static final Scalar ANNOTATE_RECT_COLOR = new Scalar(0, 255, 0);
    private final TrcDbgTrace tracer;
    private final GripPipeline gripPipeline;
    private TrcOpenCVDetector.DetectedObject[] detectedObjects = null;
    private ObjectType objectType = ObjectType.RED_CONE;

    private double totalTime = 0.0;
    private long totalFrames = 0;
    private double taskStartTime = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param imageWidth specifies the camera image width.
     * @param imageHeight specifies the camera image height.
     * @param cameraRect specifies the homography camera pixel rectangle, can be null if not provided.
     * @param worldRect specifies the homography world coordinate rectangle, can be null if not provided.
     * @param openCvCam specifies the OpenCV camera object.
     * @param cameraRotation specifies the camera orientation.
     * @param showEocvView specifies true to show the annotated image on robot controller screen, false to hide it.
     * @param tracer specifies the tracer for trace info, null if none provided.
     */
    public EocvVision(
        String instanceName, int imageWidth, int imageHeight,
        TrcHomographyMapper.Rectangle cameraRect, TrcHomographyMapper.Rectangle worldRect,
        OpenCvCamera openCvCam, OpenCvCameraRotation cameraRotation, boolean showEocvView, TrcDbgTrace tracer)
    {
        super(instanceName, imageWidth, imageHeight, cameraRect, worldRect, openCvCam, cameraRotation,
              showEocvView, tracer);

        this.tracer = tracer;
        gripPipeline = new GripPipeline();
        updatePipelineColorThreshold();
    }   //EocvVision

    /**
     * This method updates the Grip pipeline with color thresholds of the the selected object type to detect.
     */
    private void updatePipelineColorThreshold()
    {
        TrcDbgTrace.getGlobalTracer().traceInfo("updatePipelineThresholds", "objType=%s", objectType);
        switch (objectType)
        {
            case RED_CONE:
                gripPipeline.setColorThresholds(redThresholdRedCone, greenThresholdRedCone, blueThresholdRedCone);
                break;

            case BLUE_CONE:
                gripPipeline.setColorThresholds(redThresholdBlueCone, greenThresholdBlueCone, blueThresholdBlueCone);
                break;

            case YELLOW_POLE:
                gripPipeline.setColorThresholds(
                    redThresholdYellowPole, greenThresholdYellowPole, blueThresholdYellowPole);
                break;
        }
    }   //updatePipelineColorThreshold

    /**
     * This method sets the object type to detect.
     *
     * @param objType specifies the object type to detect.
     */
    public void setDetectObjectType(ObjectType objType)
    {
        objectType = objType;
        updatePipelineColorThreshold();
    }   //setDetectObjectType

    /**
     * This method sets the detect object type to the next type.
     */
    public void setNextObjectType()
    {
        objectType = ObjectType.nextObjectType(objectType);
        updatePipelineColorThreshold();
    }   //setNextObjectType

    /**
     * This method pauses/resumes pipeline processing.
     *
     * @param enabled specifies true to start pipeline processing, false to stop.
     */
    @Override
    public void setEnabled(boolean enabled)
    {
        if (enabled && !isEnabled())
        {
            detectedObjects = null;
            totalTime = 0.0;
            totalFrames = 0;
            taskStartTime = TrcUtil.getCurrentTime();
            super.setEnabled(true);
        }
        else if (!enabled && isEnabled())
        {
            super.setEnabled(false);
            detectedObjects = null;
        }
    }   //setEnabled

    /**
     * This method returns the currently detect objects in a thread safe manner.
     *
     * @return array of detected objects.
     */
    public synchronized TrcOpenCVDetector.DetectedObject[] getDetectedObjects()
    {
        TrcOpenCVDetector.DetectedObject[] targets = detectedObjects;
        detectedObjects = null;
        return targets;
    }   //getDetectedObjects

    //
    // Implements FtcEocvDetector abstract methods.
    //

    /**
     * This method is called by EasyOpenCV.OpenCVPipeline to process an image frame. It calls the grip pipeline to
     * process the image and converts the detected object into an array of TrcOpenCvDetector.DetectedObject. It also
     * annotates the original image with rectangles around the detected objects.
     *
     * @param input specifies the image frame.
     * @return annotated image frame.
     */
    @Override
    public Mat processFrame(Mat input)
    {
        final String funcName = "processFrame";
        TrcOpenCVDetector.DetectedObject[] targets = null;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.CALLBK);
        }

        double startTime = TrcUtil.getCurrentTime();
        gripPipeline.process(input);
        ArrayList<MatOfPoint> detectedTargets = gripPipeline.filterContoursOutput();
        double elapsedTime = TrcUtil.getCurrentTime() - startTime;

        totalTime += elapsedTime;
        totalFrames++;
        if (tracer != null)
        {
            tracer.traceInfo(
                funcName, "AvgProcessTime=%.3f sec, FrameRate=%.1f (Targets %sfound: %d)",
                totalTime/totalFrames, totalFrames/(TrcUtil.getCurrentTime() - taskStartTime),
                detectedTargets == null? "not ": "", detectedTargets != null? detectedTargets.size(): 0);
        }

        if (detectedTargets != null)
        {
            MatOfPoint[] contours = detectedTargets.toArray(new MatOfPoint[0]);
            targets = new TrcOpenCVDetector.DetectedObject[contours.length];
            for (int i = 0; i < targets.length; i++)
            {
                targets[i] = new TrcOpenCVDetector.DetectedObject(
                    Imgproc.boundingRect(contours[i]), Imgproc.contourArea(contours[i]));
            }
            TrcOpenCVDetector.drawRectangles(input, targets, ANNOTATE_RECT_COLOR, 0);
            synchronized (this)
            {
                detectedObjects = targets;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.CALLBK, "=%s", targets != null);
        }

        return input;
    }   //processFrame

}   //class EocvVision
