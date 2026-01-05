#include <XnOS.h>
#include <XnCppWrapper.h>

//OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std; 
using namespace xn;

// ****** Defines ******
#define KINECT_XML_PATH "Data/SamplesConfig.xml"

//Create a mask of valid depth values
IplImage *createDepthMask(IplImage *depth_image);

int main(int argc, char* argv[])
{
	//Kinect Objects
	Context niContext;
	DepthGenerator niDepth;
	ImageGenerator niImage;

	//Initialize Kinect
	EnumerationErrors errors;
	switch (XnStatus rc = niContext.InitFromXmlFile(KINECT_XML_PATH, &errors)) {
		case XN_STATUS_OK:
			break;
		case XN_STATUS_NO_NODE_PRESENT:
			XnChar strError[1024];	errors.ToString(strError, 1024);
			printf("%s\n", strError);
			return rc; break;
		default:
			printf("Open failed: %s\n", xnGetStatusString(rc));
			return rc;
	}

	//Extract the Image and Depth nodes from the Kinect Context
	niContext.FindExistingNode(XN_NODE_TYPE_DEPTH, niDepth);
	niContext.FindExistingNode(XN_NODE_TYPE_IMAGE, niImage);

	//Align the depth image and colourImage
	niDepth.GetAlternativeViewPointCap().SetViewPoint(niImage);
	niDepth.GetMirrorCap().SetMirror(false);
	//niImage.GetMirrorCap().SetMirror(false);

	//The Maximum and Minimum Kinect Depth values (used for visualisation)
	float kinectDepthMax = 8000, kinectDepthMin = 400;

	bool running = true;
	while (running) {

		if (XnStatus rc = niContext.WaitAnyUpdateAll() != XN_STATUS_OK) {
			printf("Read failed: %s\n", xnGetStatusString(rc));
			return rc;
		}

		// Update MetaData containers
		DepthMetaData niDepthMD; ImageMetaData niImageMD;
		niDepth.GetMetaData(niDepthMD); niImage.GetMetaData(niImageMD);

		// Extract Colour Image
		IplImage *colourImage = cvCreateImage(cvSize(niImageMD.XRes(), niImageMD.YRes()), IPL_DEPTH_8U, 3);
		memcpy(colourImage->imageData, niImageMD.Data(), colourImage->imageSize); cvCvtColor(colourImage, colourImage, CV_RGB2BGR);
		cvFlip(colourImage, colourImage, 1);
		cvShowImage("Colour Image", colourImage);

		// Extract Depth Image
		IplImage *depthImage = cvCreateImage(cvSize(niImageMD.XRes(), niImageMD.YRes()), IPL_DEPTH_16U, 1);
		memcpy(depthImage->imageData, niDepthMD.Data(), depthImage->imageSize);

	/*	FILE *f = fopen("dataimage.txt", "wb");
		for (int y=0; y<niImageMD.YRes(); y++) {
			for (int x=0; x<niImageMD.XRes(); x++) {
				fprintf(f, "%d\t", CV_IMAGE_ELEM(depthImage, unsigned short, y, x));
			}
			fprintf(f, "\r\n");
		}
		fclose(f);*/

		//Create a mask of the valid values for the depth image
		IplImage *depthImageMask = createDepthMask(depthImage);
		cvShowImage("Depth Image Mask", depthImageMask);

		// Convert Depth Image into visible spectrum
		float scale = 255.0/(kinectDepthMax-kinectDepthMin), shift = -kinectDepthMin*scale;
		IplImage *depthImageVisible = cvCreateImage(cvGetSize(depthImage), IPL_DEPTH_8U, 1); cvSetZero(depthImageVisible);
		cvConvertScale(depthImage, depthImageVisible, scale, shift); cvSubRS(depthImageVisible, cvScalarAll(255), depthImageVisible, depthImageMask);
		cvShowImage("Depth Image", depthImageVisible);

	//	cvSaveImage("depth.bmp", depthImageVisible);

		switch (cvWaitKey(1)) {
			case 27:
				running = false;
				break;
		}
	
		cvReleaseImage(&depthImage); cvReleaseImage(&depthImageMask);
		cvReleaseImage(&colourImage); cvReleaseImage(&depthImageVisible); 
	}

	return 0;

}

//Create a mask of valid depth values
IplImage *createDepthMask(IplImage *depthImage) {
	IplImage *depthImageMask = cvCreateImage(cvGetSize(depthImage), IPL_DEPTH_8U, 1); cvSetZero(depthImageMask);
	//Loop through each pixel in the depth image, if the depth value isn't 0, set the corresponding pixel in the mask to 255
	for (int y=0; y<depthImage->height;y++) for (int x=0; x<depthImage->width; x++)
		if (CV_IMAGE_ELEM(depthImage, unsigned short, y, x)!=0) CV_IMAGE_ELEM(depthImageMask, unsigned char, y, x)=255;

	return depthImageMask;
}