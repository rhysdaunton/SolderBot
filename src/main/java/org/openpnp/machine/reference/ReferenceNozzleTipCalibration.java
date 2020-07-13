package org.openpnp.machine.reference;

import java.awt.geom.AffineTransform;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.text.DecimalFormat;

import org.apache.commons.io.IOUtils;
import org.apache.commons.math3.geometry.spherical.twod.Circle;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.RotatedRect;
import org.openpnp.gui.MainFrame;
import org.openpnp.machine.reference.driver.GcodeDriver;
import org.openpnp.model.AbstractModelObject;
import org.openpnp.model.Configuration;
import org.openpnp.model.Length;
import org.openpnp.model.LengthUnit;
import org.openpnp.model.Location;
import org.openpnp.model.Point;
import org.openpnp.spi.Camera;
import org.openpnp.spi.Nozzle;
import org.openpnp.spi.NozzleTip;
import org.openpnp.spi.Movable.MoveToOption;
import org.openpnp.util.MovableUtils;
import org.openpnp.util.OpenCvUtils;
import org.openpnp.util.Utils2D;
import org.openpnp.util.VisionUtils;
import org.openpnp.vision.pipeline.CvPipeline;
import org.openpnp.vision.pipeline.CvStage.Result;
import org.pmw.tinylog.Logger;
import org.simpleframework.xml.Attribute;
import org.simpleframework.xml.Element;
import org.simpleframework.xml.ElementList;
import org.simpleframework.xml.ElementMap;
import org.simpleframework.xml.Root;
import org.simpleframework.xml.core.Commit;

@Root
public class ReferenceNozzleTipCalibration extends AbstractModelObject {
    public static interface RunoutCompensation {

        Location getOffset(double angle);

        Location getCameraOffset();

        Location getAxisOffset();

        @Override
        String toString();
    }

    public static class TableBasedRunoutCompensation implements ReferenceNozzleTipCalibration.RunoutCompensation {
    	@Element(required = false)
    	 
    	@ElementList(required = false)
        List<Location> nozzleTipMeasuredLocations;

        public TableBasedRunoutCompensation() {
        }
        public TableBasedRunoutCompensation(List<Location> nozzleTipMeasuredLocations) {
            //store data for later usage
            this.nozzleTipMeasuredLocations = nozzleTipMeasuredLocations;
        }

        @Override
        public Location getOffset(double angle) {

            // find the two angles in the measurements, that angle is between
            List<Location> offsets = getOffsetPairForAngle(angle);

            // the xy-offset is available via getX/getY. the angle is available via getRotation() - it's stored this way because then a simple Location type is sufficient
            Location offsetA = offsets.get(0);
            Location offsetB = offsets.get(1).convertToUnits(offsetA.getUnits());    // could this conversion be omitted?

            double ratio = 1.0;     // TODO Better solution than the workaround seems to be recommended.
            if ( (offsetB.getRotation() - offsetA.getRotation()) != 0 ) {
                ratio = (angle - offsetA.getRotation()) / (offsetB.getRotation() - offsetA.getRotation());
            }
            double deltaX = offsetB.getX() - offsetA.getX();
            double deltaY = offsetB.getY() - offsetA.getY();
            double deltaZ = offsetB.getZ() - offsetA.getZ();
            double offsetX = offsetA.getX() + (deltaX * ratio);
            double offsetY = offsetA.getY() + (deltaY * ratio);
            double offsetZ = offsetA.getZ() + (deltaZ * ratio);

            return new Location(offsetA.getUnits(), offsetX, offsetY, offsetZ, 0);
        }

        @Override
        public Location getCameraOffset() {
            return new Location(nozzleTipMeasuredLocations.get(0).getUnits());
        }

        /**
         * Find the two closest offsets to the angle being requested. The offsets start at first measurement at angleStart
         * and go to angleStop
         */
        private List<Location> getOffsetPairForAngle(double angle) {
            Location a = null, b = null;

            // angle asked for is the last in the table?

            // Make sure the angle is between -180 and 180 - angles can get larger/smaller as +-180 if limitation to 180 degrees is disabled
            //while (angle < -180) {
            //    angle += 360;
           // }
           // while (angle > 180) {
           //     angle -= 360;
           // }

            if (angle >= nozzleTipMeasuredLocations.get(nozzleTipMeasuredLocations.size() - 1).getRotation()) {
                return Arrays.asList(nozzleTipMeasuredLocations.get(nozzleTipMeasuredLocations.size() - 1), nozzleTipMeasuredLocations.get(0));
            }

            for (int i = 0; i < nozzleTipMeasuredLocations.size(); i++) {
                if (angle < nozzleTipMeasuredLocations.get(i + 1).getRotation()) {
                    a = nozzleTipMeasuredLocations.get(i);
                    b = nozzleTipMeasuredLocations.get(i + 1);
                    break;
                }
            }
            return Arrays.asList(a, b);
        }

        @Override
        public String toString() {
            return String.format(Locale.US, "%d°-offset x=%f, y=%f", (int)nozzleTipMeasuredLocations.get(0).getRotation(), nozzleTipMeasuredLocations.get(0).getX(), nozzleTipMeasuredLocations.get(0).getY());
        }

        @Override
        public Location getAxisOffset() {
            // axis offset is not available with this algorithm
            return null;
        }
    }

    public static class ModelBasedRunoutCompensation implements ReferenceNozzleTipCalibration.RunoutCompensation {
             
    	protected List<Location> nozzleTipMeasuredLocations;

        @Attribute(required = false)
        protected double centerX = 0;
        @Attribute(required = false)
        protected double centerY = 0;
        @Attribute(required = false)
        protected double radius = 0;
        @Attribute(required = false)
        protected double phaseShift;
        @Attribute(required = false)
        protected LengthUnit units = LengthUnit.Millimeters;
        @Attribute(required = false)
        protected Double peakError;
        @Attribute(required = false)
        protected Double rmsError;

        public ModelBasedRunoutCompensation() {
        }
        public ModelBasedRunoutCompensation(List<Location> nozzleTipMeasuredLocations) {
            //store data for possible later usage
            this.nozzleTipMeasuredLocations = nozzleTipMeasuredLocations;
            // save the units as the model is persisted without the locations 
            this.units = nozzleTipMeasuredLocations.size() > 0 ? 
                    nozzleTipMeasuredLocations.get(0).getUnits() : LengthUnit.Millimeters;

            // first calculate the circle fit and store the values to centerXY and radius
            // the measured offsets describe a circle with the rotational axis as the center, the runout is the circle radius
            this.calcCircleFitKasa(nozzleTipMeasuredLocations);

            // afterwards calc the phaseShift angle mapping
            this.calcPhaseShift(nozzleTipMeasuredLocations);
            
            estimateModelError(nozzleTipMeasuredLocations);

            Logger.debug("[nozzleTipCalibration]calculated nozzleEccentricity: {}", this.toString());
        }

        /**
         * Constructor that uses an affine transform to initialize the model
         * @param nozzleTipMeasuredLocations - list of measured nozzle tip locations
         * @param nozzleTipExpectedLocations - list of expected nozzle tip locations
         */
        public ModelBasedRunoutCompensation(List<Location> nozzleTipMeasuredLocations, List<Location> nozzleTipExpectedLocations) {
            // save the units as the model is persisted without the locations 
            this.units = nozzleTipMeasuredLocations.size() > 0 ? 
                    nozzleTipMeasuredLocations.get(0).getUnits() : LengthUnit.Millimeters;

            //Compute the best fit affine transform that takes the expected locations to the measured locations
            AffineTransform at = Utils2D.deriveAffineTransform(nozzleTipExpectedLocations, nozzleTipMeasuredLocations);
            Utils2D.AffineInfo ai = Utils2D.affineInfo(at);
            Logger.trace("[nozzleTipCalibration]nozzle tip affine transform = " + ai);
            
            //The expected locations were generated with a deliberate 1 mm runout so the affine scale is a direct 
            //measure of the true runout in millimeters.  However, since the affine transform gives both an x and y
            //scaling, their geometric mean is used to compute the radius.  Note that if measurement noise
            //dominates the actual runout, it is possible for the scale factors to become negative.  In
            //that case, the radius will be set to zero.
            this.radius = new Length( Math.sqrt(Math.max(0, ai.xScale) * Math.max(0, ai.yScale)),
                    LengthUnit.Millimeters).convertToUnits(this.units).getValue();
            
            //The phase shift is just the rotation of the affine transform (negated because of the subtraction in getRunout)
            this.phaseShift = -ai.rotationAngleDeg;
            
            //The center is just the translation part of the affine transform
            this.centerX = new Length( ai.xTranslation, LengthUnit.Millimeters).convertToUnits(this.units).getValue();
            this.centerY = new Length( ai.yTranslation, LengthUnit.Millimeters).convertToUnits(this.units).getValue();
            
            estimateModelError(nozzleTipMeasuredLocations);

            Logger.debug("[nozzleTipCalibration]calculated nozzleEccentricity: {}", this.toString());
        }

        /**
         * Estimates the model error based on the distance between the nozzle
         * tip measured locations and the locations computed by the model
         * @param nozzleTipMeasuredLocations - list of measured nozzle tip locations
         */
        private void estimateModelError(List<Location> nozzleTipMeasuredLocations) {
            Location peakErrorLocation = new Location(this.units);
            double sumError2 = 0;
            peakError = 0.0;
            for (Location l : nozzleTipMeasuredLocations) {
                //can't use getOffset here because it may be overridden
                Location m = getRunout(l.getRotation()).add(new Location(this.units, this.centerX, this.centerY, 0, 0));
                Logger.trace("[nozzleTipCalibration]compare measured = {}, modeled = {}", l, m);
                double error = l.convertToUnits(this.units).getLinearDistanceTo(m);
                sumError2 += error*error;
                if (error > peakError) {
                    peakError = error;
                    peakErrorLocation = l;
                }
            }
            rmsError = Math.sqrt(sumError2/nozzleTipMeasuredLocations.size());
            Logger.trace("[nozzleTipCalibration]peak error location = {}, error = {}", peakErrorLocation, peakError);
        }
        
        /* function to calc the model based runout in cartesian coordinates */
        public Location getRunout(double angle) {
            //add phase shift
            angle = angle - this.phaseShift;

            angle = Math.toRadians(angle);

            // convert from polar coords to xy cartesian offset values
            double offsetX = (this.radius * Math.cos(angle));
            double offsetY = (this.radius * Math.sin(angle));

            return new Location(this.units, offsetX, offsetY, 0, 0);
        }

        /* function to calc the model based offset in cartesian coordinates */
        @Override
        public Location getOffset(double angle) {

            Location location = getRunout(angle);

            return location.add(new Location(this.units, this.centerX, this.centerY, 0, 0));
        }

        @Override
        public Location getCameraOffset() {
            return new Location(this.units);
        }

        protected void calcCircleFitKasa(List<Location> nozzleTipMeasuredLocations) {
            /* 
             * this function fits a circle my means of the Kasa Method to the given List<Location>.
             * this is a java port of http://people.cas.uab.edu/~mosya/cl/CPPcircle.html 
             * The Kasa method should work well for this purpose since the measured locations are captured along a full circle
             */
            int n;

            double kasaXi,kasaYi,kasaZi;
            double kasaMxy,kasaMxx,kasaMyy,kasaMxz,kasaMyz;
            double kasaB,kasaC,kasaG11,kasaG12,kasaG22,kasaD1,kasaD2;
            double kasaMeanX=0.0, kasaMeanY=0.0;

            n = nozzleTipMeasuredLocations.size();

            Iterator<Location> nozzleTipMeasuredLocationsIterator = nozzleTipMeasuredLocations.iterator();
            while (nozzleTipMeasuredLocationsIterator.hasNext()) {
                Location measuredLocation = nozzleTipMeasuredLocationsIterator.next();
                kasaMeanX += measuredLocation.getX();
                kasaMeanY += measuredLocation.getY();
            }
            kasaMeanX = kasaMeanX / (double)nozzleTipMeasuredLocations.size();
            kasaMeanY = kasaMeanY / (double)nozzleTipMeasuredLocations.size();

            kasaMxx=kasaMyy=kasaMxy=kasaMxz=kasaMyz=0.;

            for (int i = 0; i < n; i++) {
                kasaXi = nozzleTipMeasuredLocations.get(i).getX() - kasaMeanX;   //  centered x-coordinates
                kasaYi = nozzleTipMeasuredLocations.get(i).getY() - kasaMeanY;   //  centered y-coordinates
                kasaZi = kasaXi*kasaXi + kasaYi*kasaYi;

                kasaMxx += kasaXi*kasaXi;
                kasaMyy += kasaYi*kasaYi;
                kasaMxy += kasaXi*kasaYi;
                kasaMxz += kasaXi*kasaZi;
                kasaMyz += kasaYi*kasaZi;
            }
            kasaMxx /= n;
            kasaMyy /= n;
            kasaMxy /= n;
            kasaMxz /= n;
            kasaMyz /= n;

            // solving system of equations by Cholesky factorization
            kasaG11 = Math.sqrt(kasaMxx);
            kasaG12 = kasaMxy / kasaG11;
            kasaG22 = Math.sqrt(kasaMyy - kasaG12 * kasaG12);

            kasaD1 = kasaMxz / kasaG11;
            kasaD2 = (kasaMyz - kasaD1*kasaG12)/kasaG22;

            // computing parameters of the fitting circle
            kasaC = kasaD2/kasaG22/2.0;
            kasaB = (kasaD1 - kasaG12*kasaC)/kasaG11/2.0;

            // assembling the output
            Double centerX = kasaB + kasaMeanX;
            Double centerY = kasaC + kasaMeanY;
            Double radius = Math.sqrt(kasaB*kasaB + kasaC*kasaC + kasaMxx + kasaMyy);

            // saving output if valid
            // the values are NaN if all given nozzleTipMeasuredLocations are the same (this is the case probably only on a simulated machine with nulldriver)
            if ( !centerX.isNaN() && !centerY.isNaN() && !radius.isNaN() ) {
                // values valid
                this.centerX = centerX;
                this.centerY = centerY;
                this.radius = radius;
            } else {
                // nozzletip has zero runout and constant offset to bottom camera
                this.centerX = nozzleTipMeasuredLocations.get(0).getX();
                this.centerY = nozzleTipMeasuredLocations.get(0).getY();
                this.radius = 0;
            }
        }

        protected void calcPhaseShift(List<Location> nozzleTipMeasuredLocations) {
            /*
             * The phaseShift is calculated to map the angle the nozzle is located mechanically at
             * (that is what openpnp shows in the DRO) to the angle, the nozzle tip is located wrt. to the
             * centered circle runout path.
             * With the phaseShift available, the calibration offset can be calculated analytically for every
             * location/rotation even if not captured while measured (stepped by angleIncrement)
             * 
             */
            double phaseShift = 0;

            double angle=0;
            double measuredAngle=0;
            double priorDifferenceAngle = 0;
            double differenceAngleMean=0;

            Iterator<Location> nozzleTipMeasuredLocationsIterator = nozzleTipMeasuredLocations.iterator();
            while (nozzleTipMeasuredLocationsIterator.hasNext()) {
                Location measuredLocation = nozzleTipMeasuredLocationsIterator.next();

                // get the measurement rotation
                angle = measuredLocation.getRotation();		// the angle at which the measurement was made was stored to the nozzleTipMeasuredLocation into the rotation attribute

                // move the offset-location by the centerY/centerY. by this all offset-locations are wrt. the 0/0 origin
                Location centeredLocation = measuredLocation.subtract(new Location(this.units,this.centerX,this.centerY,0.,0.));

                // calculate the angle, the nozzle tip is located at
                measuredAngle=Math.toDegrees(Math.atan2(centeredLocation.getY(), centeredLocation.getX()));

                // the difference is the phaseShift
                double differenceAngle = angle-measuredAngle;

                //Correct for a possible phase wrap past +/-180 degrees
                while ((priorDifferenceAngle-differenceAngle) > 180) {
                        differenceAngle += 360;
                }
                while ((priorDifferenceAngle-differenceAngle) < -180) {
                        differenceAngle -= 360;
                }
                priorDifferenceAngle = differenceAngle;
                
                Logger.trace("[nozzleTipCalibration]differenceAngle: {}", differenceAngle);

                // sum up all differenceAngles to build the average later
                differenceAngleMean += differenceAngle;
            }

            // calc the average and normalize it to +/-180 degrees
            phaseShift = Utils2D.normalizeAngle180(differenceAngleMean / nozzleTipMeasuredLocations.size());

            this.phaseShift = phaseShift;
        }


        @Override
        public String toString() {
            return String.format(Locale.US, "Center %.3f, %.3f, Runout %.3f, Phase %.3f, Peak err %.3f, RMS err %.3f %s", centerX, centerY, radius, phaseShift, peakError, rmsError, units.getShortName());
        }

        @Override
        public Location getAxisOffset() {
            return new Location(this.units,centerX,centerY,0.,0.);
        }


        public double getPhaseShift() {
            return phaseShift;
        }
        
        /**
         * @return The peak error (in this.units) of the measured nozzle tip
         * locations relative to the locations computed by the model
         */
        public Double getPeakError() {
            return peakError;
        }
        
        /**
         * @return The rms error (in this.units) of the measured nozzle tip
         * locations relative to the locations computed by the model
         */
       public Double getRmsError() {
            return rmsError;
        }
    }

    public static class ModelBasedRunoutNoOffsetCompensation extends ReferenceNozzleTipCalibration.ModelBasedRunoutCompensation {
        public ModelBasedRunoutNoOffsetCompensation() {
            super();
        }
        public ModelBasedRunoutNoOffsetCompensation(List<Location> nozzleTipMeasuredLocations) {
            super(nozzleTipMeasuredLocations);
        }
        public ModelBasedRunoutNoOffsetCompensation(List<Location> nozzleTipMeasuredLocations, List<Location> nozzleTipExpectedLocations) {
            super(nozzleTipMeasuredLocations, nozzleTipExpectedLocations);
        }

        @Override
        public String toString() {
            return String.format(Locale.US, "Camera position error %.3f, %.3f, Runout %.3f, Phase %.3f, Peak err %.3f, RMS err %.3f %s", centerX, centerY, radius, phaseShift, peakError, rmsError, units.getShortName());
        }

        @Override 
        public Location getOffset(double angle) {
            // Just return the runout, do not add the axis offset.
            return getRunout(angle);
        }

    }
    public static class ModelBasedRunoutCameraOffsetCompensation extends ReferenceNozzleTipCalibration.ModelBasedRunoutNoOffsetCompensation {
        public ModelBasedRunoutCameraOffsetCompensation() {
            super();
        }
        public ModelBasedRunoutCameraOffsetCompensation(List<Location> nozzleTipMeasuredLocations) {
            super(nozzleTipMeasuredLocations);
        }
        public ModelBasedRunoutCameraOffsetCompensation(List<Location> nozzleTipMeasuredLocations, List<Location> nozzleTipExpectedLocations) {
            super(nozzleTipMeasuredLocations, nozzleTipExpectedLocations);
        }
        
        @Override
        public String toString() {
            return String.format(Locale.US, "Camera position offset %.3f, %.3f, Runout %.3f, Phase %.3f, Peak err %.3f, RMS err %.3f %s", centerX, centerY, radius, phaseShift, peakError, rmsError, units.getShortName());
        }

        @Override
        public Location getCameraOffset() {
            // Return the axis offset as the camera tool specific calibration offset.
            Logger.debug("[nozzleTipCalibration] getCameraOffset() returns: {}, {}", this.centerX, this.centerY);
            return new Location(this.units, this.centerX, this.centerY, 0., 0.);
        }
    }


    @Element(required = false)
    private CvPipeline pipeline = createDefaultPipeline();

    @Attribute(required = false)
    private int angleSubdivisions = 6;
    @Attribute(required = false)
    private int allowMisdetections = 0;
    @Attribute(required = false)
    private double angleStart = 0;
    @Attribute(required = false)
    private double angleStop = 330;
    // The excenter radius as a ratio of the camera minimum dimension.  
    @Attribute(required = false)
    private double excenterRatio = 0.25;

    @Attribute(required = false)
    private boolean enabled;

    private boolean calibrating;

    @Deprecated
    @Element(required = false)
    private RunoutCompensation runoutCompensation = null;
    @ElementMap(required = false)
    private Map<String, RunoutCompensation> runoutCompensationLookup = new HashMap<>();

    public enum RunoutCompensationAlgorithm {
        Model, ModelAffine, ModelNoOffset, ModelNoOffsetAffine, ModelCameraOffset, ModelCameraOffsetAffine, Table
    }

    public enum RecalibrationTrigger {
        NozzleTipChange, NozzleTipChangeInJob, MachineHome,  Manual
    }

    @Attribute(required = false)
    private ReferenceNozzleTipCalibration.RunoutCompensationAlgorithm runoutCompensationAlgorithm =
        RunoutCompensationAlgorithm.ModelCameraOffsetAffine;
    
    @Attribute(required = false)
    private double version = 1.0;

    @Attribute(required = false)
    private RecalibrationTrigger recalibrationTrigger = RecalibrationTrigger.NozzleTipChangeInJob;

    /**
     * TODO Left for backward compatibility. Unused. Can be removed after Feb 7, 2020.
     */
    @Deprecated
    @Attribute(required=false)
    private Double angleIncrement = null;

    @Commit
    public void commit() {
        angleIncrement = null;
        
        // OpenPNP Version update
        if (version < 2) {
            version = 2.0;
            // Force ModelCameraOffset calibration system.
            runoutCompensationAlgorithm = RunoutCompensationAlgorithm.ModelCameraOffset;
        }
        
        //Update from KASA to Affine transform technique
        if (version < 2.1) {
            version = 2.1; //bump version number so this update is only done once
            if (runoutCompensationAlgorithm == RunoutCompensationAlgorithm.Model) {
                runoutCompensationAlgorithm = RunoutCompensationAlgorithm.ModelAffine;
            } else if (runoutCompensationAlgorithm == RunoutCompensationAlgorithm.ModelNoOffset) {
                runoutCompensationAlgorithm = RunoutCompensationAlgorithm.ModelNoOffsetAffine;
            } else if (runoutCompensationAlgorithm == RunoutCompensationAlgorithm.ModelCameraOffset) {
                runoutCompensationAlgorithm = RunoutCompensationAlgorithm.ModelCameraOffsetAffine;
            }
        }
    }

    // Max allowed linear distance w.r.t. bottom camera for an offset measurement - measurements above threshold are removed from pipelines results 
    @Attribute(required = false)
    @Deprecated
    private Double offsetThreshold = 0.0;
    @Element(required = false)
    private Length offsetThresholdLength = new Length(0.5, LengthUnit.Millimeters);
    @Element(required = false)
    private Length calibrationZOffset = new Length(0.0, LengthUnit.Millimeters);

    public ReferenceNozzleTipCalibration.RunoutCompensationAlgorithm getRunoutCompensationAlgorithm() {
        return this.runoutCompensationAlgorithm;
    }

    public void setRunoutCompensationAlgorithm(ReferenceNozzleTipCalibration.RunoutCompensationAlgorithm runoutCompensationAlgorithm) {
        this.runoutCompensationAlgorithm = runoutCompensationAlgorithm;
    }

    public void calibrate(ReferenceNozzle nozzle, boolean homing, boolean calibrateCamera) throws Exception {
        if ( !isEnabled() ) {
            return;
        }

		/*
		 * if (!(homing || Configuration.get().getMachine().isHomed())) { throw new
		 * Exception("Machine not yet homed, nozzle tip calibration request aborted"); }
		 */

        if (nozzle == null) {
            throw new Exception("Nozzle to nozzle tip mismatch.");
        }
        Camera tip_camera = VisionUtils.getBottomVisionCamera();
        Camera head_camera = nozzle.getHead().getDefaultCamera();
        ReferenceCamera referenceCamera = null;
        if (tip_camera instanceof ReferenceCamera) {
            referenceCamera = (ReferenceCamera)tip_camera;
        }
        
        HashMap<String, Object> params = new HashMap<>();
        params.put("nozzle", nozzle);
        params.put("camera", tip_camera);
        Configuration.get().getScripting().on("NozzleCalibration.Starting", params);
        
        GcodeDriver gcoder = (GcodeDriver)nozzle.getDriver();
        double nonSquareness = gcoder.getNonSquarenessFactor();
        
        //zero out the x offset as the cameras are basically in line
        Location zeroedOffset = nozzle.getHeadOffsets().derive(0d, null, null, null);
        nozzle.setHeadOffsets(zeroedOffset);
        
        MainFrame.get().getCameraViews().ensureCameraVisible(head_camera);
        MovableUtils.moveToLocationAtSafeZ(head_camera, tip_camera.getLocation());
        
        //using current offsets, calculate exact offsets
        Location cameraLocation = head_camera.getLocation();
        //center in on the dot with the head_camera
        //findUnitsPerPixel(head_camera);
        for (int i = 0; i < 3; i++) {
        	Location dotLoc = findDot(cameraLocation, head_camera);
        	head_camera.moveTo(cameraLocation.add(dotLoc));
        	cameraLocation = head_camera.getLocation();
        }
        Location measureBaseLocation = head_camera.getLocation().derive(null, null, null, 0d).add(new Location(this.calibrationZOffset.getUnits(), 0, 0, this.calibrationZOffset.getValue(), 0));
        Thread.sleep(1500);
        
        MainFrame.get().getCameraViews().ensureCameraVisible(tip_camera);
        
        int misReads = 0;
        Location tipWidth = null;
        int speed = 1000;
        Location findDotZ = new Location(nozzle.getLocation().getUnits(), 0d, 0d, 4d, 0d);
        Location zBase = null;
        do {
        	if(misReads >= 3)throw new Exception("Cannot detect a valid base tip width.");
        	
        	nozzle.moveTo(measureBaseLocation);
        
        	gcoder.sendCommand("G30"); //touch board
        	zBase = nozzle.getLocation().derive(0d, 0d, null, 0d);
        	nozzle.moveTo(nozzle.getLocation().add(findDotZ));
        	findUnitsPerPixel(tip_camera);	//must be over dot of known size, called whenever the tip camera changes Z height
        	for (int i = 0; i < 3; i++) {
        		Location dotLoc = findDot(nozzle.getLocation(), tip_camera);
        		nozzle.moveTo(nozzle.getLocation().add(dotLoc));
        	}
        
        	moveToEdge(nozzle, 0d, speed);
        	Location pinLoc = head_camera.getLocation();
        	
        	tipWidth = findTipWidth(nozzle, pinLoc, 0d, speed);
        	
        	misReads++;
        }while(Math.abs(tipWidth.getY()) < 1);
         
        double adjustedY = head_camera.getLocation().getY() - nonSquareness * head_camera.getLocation().getX();
    	Location adjustedLoc = head_camera.getLocation().derive(null, adjustedY, null, null);
    	//adjustedLoc = head_camera.getLocation();
    	Location newOffsets = measureBaseLocation.subtract(adjustedLoc).invert(true, false, false, false);
    	nozzle.setHeadOffsets(newOffsets.derive(null, null, 0d, 0d));
        Logger.debug("[nozzleTipCalibration] head offsets now set: {}", nozzle.getHeadOffsets());
        
        Thread.sleep(1000);
        
        List<Location> nozzleTipMeasuredLocations = new ArrayList<>();
        
        nozzleTipMeasuredLocations.add(new Location(LengthUnit.Millimeters, 0d, 0d, 0d, 0d));
        
        try {
            calibrating = true;
            if (! calibrateCamera) {
                reset(nozzle);
            }
            else {
                if (! isCalibrated(nozzle)) {
                    throw new Exception("Calibrate the nozzle tip first."); 
                }
                if (referenceCamera == null) {
                    throw new Exception("For calibration the bottom vision camera must be a ReferenceCamera."); 
                }
            }

            // determine the resulting angleIncrements
            double angleIncrement = ( angleStop - angleStart ) / this.angleSubdivisions;

            // determine the number of measurements to be made
            int angleSubdivisions = this.angleSubdivisions;
            if(Math.abs(angleStart + 360 - angleStop) < 0.1) {
                // we're measuring a full circle, the last measurement can be omitted
                angleSubdivisions--;
            }

            Logger.debug("[nozzleTipCalibration]starting measurement; angleStart: {}, angleStop: {}, angleIncrement: {}, angleSubdivisions: {}", 
                    angleStart, angleStop, angleIncrement, angleSubdivisions);

            // Capture nozzle tip positions and add them to a list. For these calcs the camera location is considered to be 0/0
            
            int misdetects = 0;
            Location trueTipWidth = tipWidth;
            
            for (int i = 1; i <= angleSubdivisions; i++) {            	 
            	//calc the current measurement-angle
            	double measureAngle = angleStart + (i * angleIncrement);
            	Logger.debug("[nozzleTipCalibration]i: {}, measureAngle: {}", i, measureAngle);
            	Location measureLocation = measureBaseLocation.derive(null, null, null, measureAngle);
            	Location delta = new Location(LengthUnit.Millimeters, 10d, 0d, 0d, 0d).rotateXy(measureAngle).invert(false, true, false, false);
            	Location origin = new Location(LengthUnit.Millimeters, 0d, 0d, 0d, 0d);
               	Location zOffset = null;
                misReads = 0;
                	
                do {
                    if(misReads >= 3)throw new Exception("Cannot detect a valid tip width for measure angle:  "+measureAngle);
                    	
                	nozzle.moveTo(measureLocation.subtract(delta));
                    	
                    gcoder.sendCommand("G30"); //touch board
                    
                    zOffset = nozzle.getLocation().subtract(zBase);
                    nozzle.moveTo(nozzle.getLocation().add(findDotZ));
                    for(int k = 0; k < 3; k++) {
                    	Location dotLoc = findDot(measureLocation, tip_camera);
                    	nozzle.moveTo(nozzle.getLocation().add(dotLoc));
                    }
                    	
                    moveToEdge(nozzle, measureAngle, speed);
                    Location pinLoc = head_camera.getLocation();
                    
                    tipWidth = findTipWidth(nozzle, pinLoc, measureAngle, speed);
                    
                    misReads++;
                }while(Math.abs(tipWidth.getLinearDistanceTo(origin) + trueTipWidth.getY()) > 1);
                
                adjustedY = nozzle.getLocation().getY() - nonSquareness * head_camera.getLocation().getX();
            	adjustedLoc = nozzle.getLocation().derive(null, adjustedY, null, null);
            	Location offset = measureBaseLocation.subtract(adjustedLoc);
            	offset = offset.derive(null, null, -zOffset.getZ(), measureAngle);
            	
            	Thread.sleep(1500);
                
                if (offset != null) {
            		//add final offset to array
            		nozzleTipMeasuredLocations.add(offset);
            		Logger.trace("[nozzleTipCalibration]measured offset: {}", offset);
            	}
                else {
            		misdetects++;
            		if (misdetects > this.allowMisdetections) {
            			throw new Exception("Too many vision misdetects. Check pipeline and threshold.");
            		}
            	}
            }
            
            if (nozzleTipMeasuredLocations.size() < Math.max(3, angleSubdivisions + 1 - this.allowMisdetections)) {
                throw new Exception("Not enough results from vision. Check pipeline and threshold."); 
            }
            
            this.setRunoutCompensation(nozzle, new TableBasedRunoutCompensation(nozzleTipMeasuredLocations));
            
            // go to dot (now offset-corrected). prevents the user from being irritated if it's not exactly centered
            nozzle.moveToSafeZ();
            nozzle.moveTo(measureBaseLocation.derive(null, null, Double.NaN, angleStart));
        }
        
        finally {
            // after processing the nozzle returns to safe-z
            nozzle.moveToSafeZ();
            
            Configuration.get().getScripting().on("NozzleCalibration.Finished", params);
            
            // setting to false in the very end to prevent endless calibration repetitions if calibration was not successful (pipeline not well or similar) and the nozzle is commanded afterwards somewhere else (where the calibration is asked for again ...)
            calibrating = false;
        }
    }
    
    //assumes in line vertically with pin
    private void moveToEdge(ReferenceNozzle nozzle, double measureAngle, double speed) throws Exception{
    	Camera head_camera = nozzle.getHead().getDefaultCamera();
    	GcodeDriver gcoder = (GcodeDriver)nozzle.getDriver();
    	double nonSquarenessFactor = gcoder.getNonSquarenessFactor();
    	DecimalFormat decimalFormat = new DecimalFormat("0.0000");
    	//find the edge
    	Location machineLoc = head_camera.getLocation();
    	Location delta = new Location(LengthUnit.Millimeters, 15d, 0d, 0d, 0d).rotateXy(measureAngle).invert(false, true, false, false);
        Location calMove = machineLoc.add(delta);
        double x = calMove.getX();
    	double y = calMove.getY() + nonSquarenessFactor * x;
    	String command = "G38.3 X"+decimalFormat.format(x)+" Y"+decimalFormat.format(y)+" F"+speed;
    	gcoder.sendCommand(command);
    	gcoder.sendCommand("M400");
    	return;
    }
    
    //assumes touching tip not on either end
    private void moveToTipEnd(String end, ReferenceNozzle nozzle, Location pinLoc, double measureAngle, double speed) throws Exception{
    	Camera head_camera = nozzle.getHead().getDefaultCamera();
    	GcodeDriver gcoder = (GcodeDriver)nozzle.getDriver();
    	DecimalFormat decimalFormat = new DecimalFormat("0.0000");
    	Location pushIn = new Location(LengthUnit.Millimeters, 0.5, 0d, 0d, 0d).rotateXy(measureAngle).invert(false, true, false, false);
    	double startZ = nozzle.getLocation().getZ();
    	Location delta = null;
    	
    	if (end == "bottom") {
    		delta = new Location(LengthUnit.Millimeters, 0d, -5d, 0d, 0d).rotateXy(measureAngle).invert(false, true, false, false);
    	}
    	else if(end == "top") {
    		delta = new Location(LengthUnit.Millimeters, 0d, 5d, 0d, 0d).rotateXy(measureAngle).invert(false, true, false, false);
    	}
    	else {
    		throw new Exception("Tip end not specified");
    	}
    	
    	//safe Z
    	Location calMove = pinLoc.add(delta).add(pushIn);
    	double z = calMove.getZ();
    	String command = "G1 Z"+decimalFormat.format(z)+" F"+speed;
    	gcoder.sendCommand(command);
    	gcoder.sendCommand("M400");
    	
    	//move away
    	double x = calMove.getX();
    	double y = calMove.getY();
    	command = "G1 X"+decimalFormat.format(x)+" Y"+decimalFormat.format(y)+" F"+speed;
    	gcoder.sendCommand(command);
    	gcoder.sendCommand("M400");
    	
    	//start Z
    	z = startZ;
    	command = "G1 Z"+decimalFormat.format(z)+" F"+speed;
    	gcoder.sendCommand(command);
    	gcoder.sendCommand("M400");
    	
    	//move to pin
    	calMove = pinLoc.subtract(delta);
    	x = calMove.getX();
    	y = calMove.getY();
    	command = "G38.3 X"+decimalFormat.format(x)+" Y"+decimalFormat.format(y)+" F"+speed;
    	gcoder.sendCommand(command);
    	gcoder.sendCommand("M400");
    	
    	calMove = head_camera.getLocation().subtract(pushIn);
    	x = calMove.getX();
    	y = calMove.getY();
    	command = "G1 X"+decimalFormat.format(x)+" Y"+decimalFormat.format(y)+" F"+speed;
    	gcoder.sendCommand(command);
    	gcoder.sendCommand("M400");
    	return;
    }
    
    //assumes already touching tip
    private Location findTipWidth(ReferenceNozzle nozzle, Location pinLoc, double measureAngle, double speed) throws Exception {
    	Camera head_camera = nozzle.getHead().getDefaultCamera();
    	GcodeDriver gcoder = (GcodeDriver)nozzle.getDriver();
    	DecimalFormat decimalFormat = new DecimalFormat("0.0000");
    	
    	moveToTipEnd("top", nozzle, pinLoc, measureAngle, speed);
    	Location topTip = head_camera.getLocation();
    	
    	moveToTipEnd("bottom", nozzle, pinLoc, measureAngle, speed);
    	Location bottomTip = head_camera.getLocation();
    	
    	Location tipWidth = topTip.subtract(bottomTip);
    	Location tipHalfWidth = tipWidth.multiply(new Location(LengthUnit.Millimeters, 0d, 0.5d, 0d, 0d).rotateXy(measureAngle).absolute(true, true, true, true));
    	Location centerTip = bottomTip.add(tipHalfWidth);
    	Location calMove = centerTip;
    	double x = calMove.getX();
    	double y = calMove.getY();
    	String command = "G1 X"+decimalFormat.format(x)+" Y"+decimalFormat.format(y)+" F"+speed;
    	gcoder.sendCommand(command);
    	gcoder.sendCommand("M400");
    	gcoder.sendCommand("M114");

    	return tipWidth;
    }

    public static void resetAllNozzleTips() {
        // Reset all nozzle tip calibrations, as they have become invalid due to some machine configuration change.
        for (NozzleTip nt: Configuration.get().getMachine().getNozzleTips()) {
            if (nt instanceof ReferenceNozzleTip) {
                ((ReferenceNozzleTip)nt).getCalibration().resetAll();
            }
        }
    }

    public void calibrate(ReferenceNozzle nozzle) throws Exception {
        calibrate(nozzle, false, false);
    }

    public void calibrateCamera(ReferenceNozzle nozzle) throws Exception {
        calibrate(nozzle, false, true);
    }

    /*
     * While calibrating the nozzle a circle was fitted to the runout path of the tip.
     * here the offset is reconstructed in XY-cartesian coordinates to be applied in moveTo commands.
     */
    public Location getCalibratedOffset(ReferenceNozzle nozzle, double angle) {
        if (!isEnabled() || !isCalibrated(nozzle)) {
            return new Location(LengthUnit.Millimeters, 0, 0, 0, 0);
        }

        return this.getRunoutCompensation(nozzle).getOffset(angle);

    }

    /*
     * The axis offset determined in runout calibration can be applied as a tool specific camera offset.
     */
    public Location getCalibratedCameraOffset(ReferenceNozzle nozzle, Camera camera) {
        try {
            if (camera == VisionUtils.getBottomVisionCamera()) {
                if (isEnabled() && isCalibrated(nozzle)) {
                    return this.getRunoutCompensation(nozzle).getCameraOffset();
                }
            } 
        }
        catch (Exception e) {
            // There is no bottom vision camera, that's fine.
        }

        return new Location(LengthUnit.Millimeters, 0, 0, 0, 0);
    }
    
    private void findUnitsPerPixel(Camera camera) throws Exception{
		try(CvPipeline pipeline = getPipeline()){
			pipeline.setProperty("camera", camera);
			
			pipeline.process();
	        Result result = pipeline.getResult(VisionUtils.PIPELINE_RESULTS_NAME);
	        if (result == null || result.model == null) {
	            return;
	        }
	        List<Result.Circle> circles = (List<Result.Circle>) result.model;
	        Result.Circle center = circles.get(0);
	        double rawDiameter = center.getDiameter();
	        double knownDiameter = 2.75; //mm
	        double ratio = knownDiameter/rawDiameter;
	        Location newUnitsPerPixel = new Location(camera.getLocation().getUnits(), ratio, ratio, 0d, 0d); //assume square pixels
	        camera.setUnitsPerPixel(newUnitsPerPixel);
	        Logger.debug("Units per pixel of: {} set on camera: {}", newUnitsPerPixel, camera);
		}
		return;
    }
    
    private Location findTip(Camera camera) throws Exception{
    	try(CvPipeline pipeline = getPipeline()){
    		pipeline.setProperty("camera", camera);
    		
    		pipeline.process();
    		List<Location> locations = new ArrayList<>();
    		
    		String stageName = "tip_results";
    		Result pipelineResult = pipeline.getResult(stageName);
            if (pipelineResult == null) {
                throw new Exception(String.format("There should be a \"%s\" stage in the pipeline.", stageName));
            }

            Object results = pipelineResult.model;
            
            if (results instanceof Exception) {
                throw (Exception)results;
            }
     
            KeyPoint keyPoint = ((KeyPoint) results);
            locations.add(VisionUtils.getPixelCenterOffsets(camera, keyPoint.pt.x, keyPoint.pt.y));
            
            Location result = locations.get(0);//.add(new Location(LengthUnit.Millimeters, 0.75d, 0d, 0d, 0d));
            
            return result;
    	}
    	finally {
    		pipeline.setProperty("MaskCircle.center", null);
    	}
    }
    
    private Location findDot(Location measureLocation, Camera camera) throws Exception {
    	try (CvPipeline pipeline = getPipeline()){
    		pipeline.setProperty("camera", camera);
    		
    		pipeline.process();
    		List<Location> locations = new ArrayList<>();
    		
    		String stageName = VisionUtils.PIPELINE_RESULTS_NAME;
    		Result pipelineResult = pipeline.getResult(stageName);
            if (pipelineResult == null) {
                throw new Exception(String.format("There should be a \"%s\" stage in the pipeline.", stageName));
            }

            Object results = pipelineResult.model;
            
            if (results instanceof Exception) {
                throw (Exception)results;
            }
            
            //show result from pipeline in camera view
            MainFrame.get().getCameraViews().getCameraView(camera).showFilteredImage(
                    OpenCvUtils.toBufferedImage(pipeline.getWorkingImage()), 1000);
            
         // add all results from pipeline to a Location-list post processing
            if (results instanceof List) {
                // are there any results from the pipeline?
                if (0==((List) results).size()) {
                    // Don't throw new Exception("No results from vision. Check pipeline.");      
                    // Instead the number of obtained fixes is evaluated later.
                    return null;
                }
                for (Object result : (List) results) {
                    if ((result) instanceof Result.Circle) {
                        Result.Circle circle = ((Result.Circle) result);
                        locations.add(VisionUtils.getPixelCenterOffsets(camera, circle.x, circle.y));
                    }
                    else if ((result) instanceof KeyPoint) {
                        KeyPoint keyPoint = ((KeyPoint) result);
                        locations.add(VisionUtils.getPixelCenterOffsets(camera, keyPoint.pt.x, keyPoint.pt.y));
                    }
                    else if ((result) instanceof RotatedRect) {
                        RotatedRect rect = ((RotatedRect) result);
                        locations.add(VisionUtils.getPixelCenterOffsets(camera, rect.center.x, rect.center.y));
                    }
                    else {
                        throw new Exception("Unrecognized result " + result);
                    }
                }
            }
            //rotate by -camera angle
            return locations.get(0).rotateXy(-(measureLocation.getRotation()));
    	}
    	finally {
            pipeline.setProperty("MaskCircle.center", null);
        }
    }

    private Location findCircle(Location measureLocation) throws Exception {
        Camera camera = VisionUtils.getBottomVisionCamera();
        try (CvPipeline pipeline = getPipeline()) {
            pipeline.setProperty("camera", camera);
            Point maskCenter = VisionUtils.getLocationPixels(camera, measureLocation);
            pipeline.setProperty("MaskCircle.center", new org.opencv.core.Point(maskCenter.getX(), maskCenter.getY()));

            pipeline.process();
            List<Location> locations = new ArrayList<>();

            String stageName = VisionUtils.PIPELINE_RESULTS_NAME;
            Result pipelineResult = pipeline.getResult(stageName);
            if (pipelineResult == null) {
                throw new Exception(String.format("There should be a \"%s\" stage in the pipeline.", stageName));
            }

            Object results = pipelineResult.model;

            if (results instanceof Exception) {
                throw (Exception)results;
            }

            //show result from pipeline in camera view
            MainFrame.get().getCameraViews().getCameraView(camera).showFilteredImage(
                    OpenCvUtils.toBufferedImage(pipeline.getWorkingImage()), 1000);

            // add all results from pipeline to a Location-list post processing
            if (results instanceof List) {
                // are there any results from the pipeline?
                if (0==((List) results).size()) {
                    // Don't throw new Exception("No results from vision. Check pipeline.");      
                    // Instead the number of obtained fixes is evaluated later.
                    return null;
                }
                for (Object result : (List) results) {
                    if ((result) instanceof Result.Circle) {
                        Result.Circle circle = ((Result.Circle) result);
                        locations.add(VisionUtils.getPixelCenterOffsets(camera, circle.x, circle.y));
                    }
                    else if ((result) instanceof KeyPoint) {
                        KeyPoint keyPoint = ((KeyPoint) result);
                        locations.add(VisionUtils.getPixelCenterOffsets(camera, keyPoint.pt.x, keyPoint.pt.y));
                    }
                    else if ((result) instanceof RotatedRect) {
                        RotatedRect rect = ((RotatedRect) result);
                        locations.add(VisionUtils.getPixelCenterOffsets(camera, rect.center.x, rect.center.y));
                    }
                    else {
                        throw new Exception("Unrecognized result " + result);
                    }
                }
            }

            // remove all results that are above threshold
            Iterator<Location> locationsIterator = locations.iterator();
            while (locationsIterator.hasNext()) {
                Location location = locationsIterator.next();
                Location measureLocationRelative = measureLocation.convertToUnits(location.getUnits()).
                        subtract(camera.getLocation());
                double threshold = offsetThresholdLength.convertToUnits(location.getUnits()).getValue();
                if (location.getLinearDistanceTo(measureLocationRelative) > threshold) {
                    locationsIterator.remove();
                    Logger.trace("[nozzleTipCalibration]Removed offset location {} from results; measured distance {} exceeds offsetThresholdLength {}", location, location.getLinearDistanceTo(0., 0.), threshold); 
                }
            }

            // check for a valid resultset
            if (locations.size() == 0) {
                // Don't throw new Exception("No valid results from pipeline within threshold");
                // Instead the number of obtained fixes is evaluated later.
                return null;
            } else if (locations.size() > 1) {
                // Don't throw an exception here either. Since we've gotten more results than expected we can't be
                // sure which, if any, are the correct result so just discard them all and log an info message.
                Logger.info("[nozzleTipCalibration]Got more than one result from pipeline. For best performance tweak pipeline to return exactly one result only. Discarding all locations (since it is unknown which may be correct) from the following set: " + locations);
                return null;
            }

            // finally return the location at index (0) which is either a) the only one or b) the one best matching the nozzle tip
            return locations.get(0);
        }
        finally {
            pipeline.setProperty("MaskCircle.center", null);
        }
    }

    public static CvPipeline createDefaultPipeline() {
        try {
            String xml = IOUtils.toString(ReferenceNozzleTip.class
                    .getResource("ReferenceNozzleTip-Calibration-DefaultPipeline.xml"));
            return new CvPipeline(xml);
        }
        catch (Exception e) {
            throw new Error(e);
        }
    }

    public void resetPipeline() {
        pipeline = createDefaultPipeline();
    }

    public void reset(ReferenceNozzle nozzle) {
        // reset the combined nozzle tip + nozzle runout compensation for the nozzle we are currently attached to 
        setRunoutCompensation(nozzle, null);
        // deprecated
        runoutCompensation = null;
    }

    public void resetAll() {
        // reset the nozzle tip + nozzle runout for all the nozzles this tip was attached to
        // i.e. just wipe the whole lookup table
        runoutCompensationLookup.clear();
        // inform UI about changed information
        firePropertyChange("calibrationInformation", null, null);
        // deprecated
        runoutCompensation = null;
    }

    private RunoutCompensation getRunoutCompensation(ReferenceNozzle nozzle) {
        // get the combined nozzle tip + nozzle runout compensation for the nozzle we are currently attached to 
        if (nozzle != null) {
            return runoutCompensationLookup.get(nozzle.getId());
        }
        return null;
    }

    private void setRunoutCompensation(ReferenceNozzle nozzle, RunoutCompensation runoutCompensation) {
        // set the combined nozzle tip + nozzle runout compensation for the nozzle we are currently attached to 
        if (nozzle != null) {
            if (runoutCompensation == null) {
                runoutCompensationLookup.remove(nozzle.getId());
            }
            else {
                runoutCompensationLookup.put(nozzle.getId(), runoutCompensation);
            }
                
            // inform UI about changed information
            firePropertyChange("calibrationInformation", null, null);
        }
        // deprecated
        runoutCompensation = null;
    }

    public String getCalibrationInformation(ReferenceNozzle nozzle) { 
        return getRunoutCompensation(nozzle).toString();
    }

    public boolean isCalibrated(ReferenceNozzle nozzle) {
        return getRunoutCompensation(nozzle) != null;
    }

    public boolean isCalibrating() {
        return calibrating;
    }

    public int getAngleSubdivisions() {
        return angleSubdivisions;
    }

    public void setAngleSubdivisions(int angleSubdivisions) {
        this.angleSubdivisions = angleSubdivisions;
    }

    public int getAllowMisdetections() {
        return allowMisdetections;
    }

    public void setAllowMisdetections(int allowMisdetections) {
        this.allowMisdetections = allowMisdetections;
    }

    @Deprecated
    public double getOffsetThreshold() {
        return getOffsetThresholdLength().convertToUnits(LengthUnit.Millimeters).getValue();
    }

    @Deprecated
    public void setOffsetThreshold(double offsetThreshold) {
        this.setOffsetThresholdLength(new Length(offsetThreshold, LengthUnit.Millimeters));
    }

    public Length getOffsetThresholdLength() {
        // Migrate old unit-less setting.
        if (this.offsetThreshold > 0.) {
            offsetThresholdLength = new Length(this.offsetThreshold, LengthUnit.Millimeters);
            this.offsetThreshold = 0.;
        }
        return offsetThresholdLength;
    }

    public void setOffsetThresholdLength(Length offsetThresholdLength) {
        Length oldValue = this.offsetThresholdLength;
        this.offsetThresholdLength = offsetThresholdLength;
        firePropertyChange("offsetThresholdLength", oldValue, offsetThresholdLength);
    }

    public Length getCalibrationZOffset() {
        return calibrationZOffset;
    }

    public void setCalibrationZOffset(Length calibrationZOffset) {
        this.calibrationZOffset = calibrationZOffset;
    }

    public RecalibrationTrigger getRecalibrationTrigger() {
        return recalibrationTrigger;
    }

    public void setRecalibrationTrigger(RecalibrationTrigger recalibrationTrigger) {
        this.recalibrationTrigger = recalibrationTrigger;
    }

    public boolean isRecalibrateOnNozzleTipChangeInJobNeeded(ReferenceNozzle nozzle) {
        return recalibrationTrigger == RecalibrationTrigger.NozzleTipChangeInJob;
    }

    public boolean isRecalibrateOnNozzleTipChangeNeeded(ReferenceNozzle nozzle) {
        return (recalibrationTrigger == RecalibrationTrigger.NozzleTipChange)
                || (recalibrationTrigger == RecalibrationTrigger.MachineHome && !isCalibrated(nozzle));
    }

    public boolean isRecalibrateOnHomeNeeded(ReferenceNozzle nozzle) {
        return recalibrationTrigger == RecalibrationTrigger.NozzleTipChange
                ||  recalibrationTrigger == RecalibrationTrigger.MachineHome;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public CvPipeline getPipeline() throws Exception {
        pipeline.setProperty("camera", VisionUtils.getBottomVisionCamera());
        return pipeline;
    }

    public void setPipeline(CvPipeline calibrationPipeline) {
        this.pipeline = calibrationPipeline;
    }
}