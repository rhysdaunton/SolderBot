package org.openpnp.vision.pipeline.stages;


import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.openpnp.vision.pipeline.CvPipeline;
import org.openpnp.vision.pipeline.CvStage;
import org.openpnp.vision.pipeline.Stage;
import org.openpnp.vision.pipeline.CvStage.Result;
import org.simpleframework.xml.Attribute;

@Stage(category = "Image Processing",
        description = "Find the center of the solder tip.")

public class FindTip extends CvStage {
    @Attribute(required = false)
    private String imageStageName = null;

    public String getImageStageName() {
        return imageStageName;
    }

    public void setImageStageName(String imageStageName) {
        this.imageStageName = imageStageName;
    }

    @Override
    public Result process(CvPipeline pipeline) throws Exception {
        if (imageStageName == null) {
            return null;
        }
        Result image = pipeline.getResult(imageStageName);
        if (image == null || image.image == null) {
            return null;
        }
        Mat img = image.getImage();
        
        byte[] imgData = new byte[(int) (img.total() * img.channels())];
        img.get(0, 0, imgData);
        int halfHeight = img.rows()/2;
        int x;
        boolean hit = false;
        for(x = 0; x < img.cols(); x++) {
        	int intensity = imgData[halfHeight * img.cols() + x];
        	if(intensity == -1) {
        		hit = true;
        		break;
        	}
        }
        if(hit) {
        	KeyPoint point = new KeyPoint(x, halfHeight, 4);
            return new Result(null, point);
        }
        return null;
    }
}