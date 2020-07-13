package org.openpnp.util;

import org.openpnp.machine.reference.ReferenceCamera;
import org.openpnp.machine.reference.ReferenceDriver;
import org.openpnp.machine.reference.ReferenceMachine;
import org.openpnp.machine.reference.ReferenceNozzle;
import org.openpnp.machine.reference.driver.GcodeDriver;
import org.openpnp.model.Configuration;
import org.openpnp.model.Length;
import org.openpnp.model.LengthUnit;
import org.openpnp.model.Location;
import org.openpnp.spi.Head;
import org.openpnp.spi.HeadMountable;
import org.openpnp.spi.Nozzle;
import org.openpnp.spi.Locatable;
import org.openpnp.spi.Movable.MoveToOption;

public class MovableUtils {
    /**
     * Moves the given HeadMountable to the specified Location by first commanding the head to
     * safe-Z all of it's components, then moving the HeadMountable in X, Y and C, followed by
     * moving in Z.
     * 
     * @param hm
     * @param location
     * @param speed
     * @throws Exception
     */
    public static void moveToLocationAtSafeZ(HeadMountable hm, Location location, double speed)
            throws Exception {
        Head head = hm.getHead();
        if(hm.getName().equals("FID Spotter")) {
        	ReferenceCamera referenceCamera = (ReferenceCamera) head.getDefaultCamera();
            ReferenceNozzle referenceNozzle = (ReferenceNozzle) head.getDefaultNozzle();
            Length nozzleSafeZ = referenceNozzle.getSafeZ();
            Length cameraSafeZ = referenceCamera.getSafeZ();
            referenceNozzle.setSafeZ(cameraSafeZ);
        	head.moveToSafeZ(speed);
        	Nozzle nozzle = head.getDefaultNozzle();
        	nozzle.moveTo(location.derive(Double.NaN, Double.NaN, Double.NaN, 0d));	//zero the rotation
        	referenceNozzle.setSafeZ(nozzleSafeZ);
        }
        else {
        	head.moveToSafeZ(speed);
        }
        hm.moveTo(location.derive(null, null, Double.NaN, null));
       	hm.moveTo(location, speed);
    }
    
    public static void moveToLocationAtSafeZ(HeadMountable hm, Location location, double speed, boolean slide)throws Exception {
    	Head head = hm.getHead();
    	head.moveToSafeZ(speed);
    	if (slide) {
    		double angle = location.getRotation();
    		double slideRadius = 1.7;
    		Location slideDist = new Location(LengthUnit.Millimeters, slideRadius, 0d, -slideRadius, 0d).rotateXy(angle).invert(false,true,false,false);
    		hm.moveTo(location.derive(null, null, Double.NaN, null), speed);
    		hm.moveTo(location.subtract(slideDist), speed);							//move to part 1.7 mm away
        	hm.moveTo(location, speed * 0.05);										//drop to Z
    	}
    	else {
    		hm.moveTo(location.derive(null, null, Double.NaN, null));
        	hm.moveTo(location, speed);
    	}
    }
    
    public static void moveToLocationAtSafeZ(HeadMountable hm, Location location, double speed, boolean slide, boolean preWet)throws Exception {
    	Head head = hm.getHead();
    	head.moveToSafeZ(speed);
    	if (slide) {
    		double angle = location.getRotation();
    		double slideRadius = 1.7;
    		
    		Location slideDist = new Location(LengthUnit.Millimeters, slideRadius, 0d, -slideRadius, 0d).rotateXy(angle).invert(false,true,false,false);
    		hm.moveTo(location.derive(null, null, Double.NaN, null), speed);
    		if(preWet) {
    			hm.moveTo(location.subtract(slideDist), speed, MoveToOption.PreWet);
    		}
    		else {
    			hm.moveTo(location.subtract(slideDist), speed);
    		}
        	hm.moveTo(location, speed * 0.05);										//drop to Z
    	}
    	else {
    		hm.moveTo(location.derive(null, null, Double.NaN, null));
        	hm.moveTo(location, speed);
    	}
    }
    
    static ReferenceDriver getDriver() {
        return getMachine().getDriver();
    }

    static ReferenceMachine getMachine() {
        return (ReferenceMachine) Configuration.get().getMachine();
    }

    public static void moveToLocationAtSafeZ(HeadMountable hm, Location location) throws Exception {
        moveToLocationAtSafeZ(hm, location, hm.getHead().getMachine().getSpeed());
    }
    
    public static void moveToLocationAtSafeZ(HeadMountable hm, Location location, boolean slide) throws Exception{
    	moveToLocationAtSafeZ(hm, location, hm.getHead().getMachine().getSpeed(), slide);
    }
    
    public static void moveToLocationAtSafeZ(HeadMountable hm, Location location, boolean slide, boolean preWet) throws Exception{
    	moveToLocationAtSafeZ(hm, location, hm.getHead().getMachine().getSpeed(), slide, preWet);
    }
    
    public static void park(Head head) throws Exception {
        head.moveToSafeZ();
        HeadMountable hm = head.getDefaultCamera();
        Location location = head.getParkLocation();
        location = location.derive(null, null, Double.NaN, Double.NaN);
        hm.moveTo(location);
    }
}
