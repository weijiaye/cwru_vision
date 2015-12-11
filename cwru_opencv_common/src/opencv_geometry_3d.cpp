#include <ros/ros.h>
#include "cwru_opencv_common/opencv_geometry_3d.h"
#include "cwru_opencv_common/projective_geometry.h"



using namespace cv;


namespace cv_3d {



    Rect renderSphere(cv::Mat & inputImage,const sphere sphereIn, const cv::Mat &P, OutputArray centerPt,  OutputArray jac )
    {
        //project the center;
        Point2d center(0.0,0.0);
        if(jac.needed())
        {

            center = cv_projective::reprojectPoint(sphereIn.center,P,cv::Mat(),cv::Mat(),jac);

        }
        else
        {
            center = cv_projective::reprojectPoint(sphereIn.center,P);
        }


        //estimate the radius:
        //This estimate technically has a radius jacobian as well...
        /* 
         * @todo Look into adding a radial jacobian
         */
        //Aproximate f:
        Mat subP = P.colRange(0,3);

        double P_det = determinant(subP);

        //ROS_INFO_STREAM(P);

        double fEst = sqrt(abs(P_det));

        //approximate the distance.
        Mat pointTemp(4,1,CV_64FC1);
        pointTemp.at<double>(0) = sphereIn.center.x;
        pointTemp.at<double>(1) = sphereIn.center.y;
        pointTemp.at<double>(2) = sphereIn.center.z;
        pointTemp.at<double>(3) = 1.0;


        Mat projected = P*pointTemp;

        double distEst = projected.at<double>(2);

        //estimate the radius.
        double radEst = sphereIn.radius*fEst/distEst;

        Point drawCenter(center.x,center.y);
        int radius = (int) radEst;
        //ROS_INFO("Sphere Radius is %f : %d\n",radEst,radius);
        if(centerPt.needed())
        {
            ROS_INFO("Getting the center Pt and Radius");
            centerPt.create(3,1,CV_64FC1);
            Mat output(centerPt.getMat());

            output.at<double>(0) = center.x;
            output.at<double>(1) = center.y;
            output.at<double>(2) = radEst;

        }

        //default is a filled white circle.
        circle(inputImage,drawCenter,radius,Scalar(255,255,255),-1);

        //output bounding box (used for more efficient code execution (optional);
        Rect boundingBox;
        boundingBox.x = drawCenter.x-radius;
        boundingBox.y = drawCenter.y-radius;
        boundingBox.width = 2*radius;
        boundingBox.height = 2*radius;
        return boundingBox;
    }

    RotatedRect renderCylinder(cv::Mat & inputImage,const cylinder cylinderIn, const cv::Mat &P,OutputArray tips, OutputArray jac  )
    {


        Mat jac_dir;

        Point3d localDir(0.0,0.0,0.0);

        if(jac.needed())
        {
            localDir= computeNormalFromSpherical(cylinderIn.theta,cylinderIn.phi,jac_dir);
        }
        else
        {
            localDir= computeNormalFromSpherical(cylinderIn.theta,cylinderIn.phi);
        }

        ROS_INFO("Computing JAC");

        //negative normal point.
        Mat end0(4,1,CV_64FC1);
        end0.at<double>(0) = cylinderIn.center.x-localDir.x*cylinderIn.height/2;
        end0.at<double>(1) = cylinderIn.center.y-localDir.y*cylinderIn.height/2;
        end0.at<double>(2) = cylinderIn.center.z-localDir.z*cylinderIn.height/2;
        end0.at<double>(3) = 1.0;

        //positive normal point.
        Mat end1(4,1,CV_64FC1);
        end1.at<double>(0) = cylinderIn.center.x+localDir.x*cylinderIn.height/2;
        end1.at<double>(1) = cylinderIn.center.y+localDir.y*cylinderIn.height/2;
        end1.at<double>(2) = cylinderIn.center.z+localDir.z*cylinderIn.height/2;
        end1.at<double>(3) = 1.0;



        Point3d end0Pt(cylinderIn.center-localDir*(cylinderIn.height/2.0));
        Point3d end1Pt(cylinderIn.center+localDir*(cylinderIn.height/2.0));


        Mat jac_0;
        Mat jac_1;

        Point2d pt0(0.0,0.0);
        Point2d pt1(0.0,0.0);

        //pt0 is the endpoint in the positive direction
        //pt1 is the endpoint in the negative direction
        // (this is important for the torque direction.

        if(jac.needed())
        {

            pt0 = cv_projective::reprojectPoint(end0Pt,P,cv::Mat(),cv::Mat(),jac_0);
            pt1 = cv_projective::reprojectPoint(end1Pt,P,cv::Mat(),cv::Mat(),jac_1);

        }
        else
        {
            pt0 = cv_projective::reprojectPoint(end0Pt,P,cv::Mat(),cv::Mat(),jac_0);
            pt1 = cv_projective::reprojectPoint(end1Pt,P,cv::Mat(),cv::Mat(),jac_1);
        }

      
        Mat projected0 = P*end0;
        Mat projected1 = P*end1;

        Mat subP = P.colRange(0,3);
        double P_det = determinant(subP);
        double fEst = sqrt(abs(P_det));

        double distEst0(projected0.at<double>(2));
        double distEst1(projected1.at<double>(2));

        //estimate the radius.
        double radEst0 = cylinderIn.radius*fEst/distEst0;
        double radEst1 = cylinderIn.radius*fEst/distEst1;


        //create the set of 4 points.
        Point corners[4];

        Point2d dir = pt1-pt0;

        double dirLength = norm(dir);


        //draw the sphere at this point.
        if(dirLength < radEst0)
        {
            dir *= (1/norm(dir));

        }

        Point2d radDir(-dir.y,dir.x);

        Point2d c0 = pt0+radDir*radEst0;
        Point2d c1 = pt1+radDir*radEst1;
        Point2d c2 = pt1-radDir*radEst1;
        Point2d c3 = pt0-radDir*radEst0;

        if(tips.needed())
        {

            tips.create(6,1,CV_64FC1);
            Mat tipMat(tips.getMat());

            tipMat.at<double>(0) = pt0.x;
            tipMat.at<double>(1) = pt0.x;

            tipMat.at<double>(2) = radEst0;

            tipMat.at<double>(3) = pt1.x;
            tipMat.at<double>(4) = pt1.y;

            tipMat.at<double>(5) = radEst1;


        }

        std::vector< Point > cornersV;
        cornersV.resize(4);
        cornersV[0] = corners[0] = Point(c0.x,c0.y);
        cornersV[1] = corners[1] = Point(c1.x,c1.y);
        cornersV[2] = corners[2] = Point(c2.x,c2.y);
        cornersV[3] = corners[3] = Point(c3.x,c3.y);

        //Draw the 4 points in the image.
        fillConvexPoly(inputImage,corners,4,Scalar(255,255,255),CV_AA);

        /*
         * @todo create the local jacobian
         *  define the jacobian in terms of a rotation and translation. due to the position of the points.
         *  2d pose derivative.
         */
        if(jac.needed())
        {
            ROS_INFO("Making tip Mat");
            Mat dEnd0(3,5,CV_64FC1);
            Mat dEnd1(3,5,CV_64FC1);

            ((Mat) (jac_dir*cylinderIn.height/2)).copyTo(dEnd1.colRange(3,5));
            ((Mat) (jac_dir*-cylinderIn.height/2)).copyTo(dEnd0.colRange(3,5));

        ((Mat)Mat::eye(3,3,CV_64FC1)).copyTo(dEnd0.colRange(0,3));
        ((Mat)Mat::eye(3,3,CV_64FC1)).copyTo(dEnd1.colRange(0,3));

            
            ROS_INFO("Making output Jac Mat");
            //center point derivative
            //derivative is a function of the cylinderical motion.
            Mat jac_ends(4,5,CV_64FC1);

            //combine the two derivatives.
            ((Mat) (jac_0*dEnd0)).copyTo((Mat) jac_ends.rowRange(0,2));
            ((Mat) (jac_1*dEnd1)).copyTo((Mat) jac_ends.rowRange(2,4));

             ROS_INFO("Making output Jac Mat");
            jac.create(4,5,CV_64FC1);

            jac_ends.copyTo(jac.getMat());
            ROS_INFO("Finished jacobian generation");
        }

        return minAreaRect(cornersV);
    }




    void optimizeSphere(sphere &sphereIn, const Mat& segmentedImage, const Mat& P_l, const Mat& P_r , int k_width, double k_var,bool displayPause)
    {
        /*
         * @todo Finish this function
         */

        Mat segmentedImageFloat;
        segmentedImage.convertTo(segmentedImageFloat, CV_32FC1);
        Mat mask_l(Mat::zeros(segmentedImage.size(),segmentedImageFloat.type()));
        Mat mask_r(Mat::zeros(segmentedImage.size(),segmentedImageFloat.type()));

        Mat jac_l,jac_r;
        Mat center_l,center_r;
        Rect sphereInBox_l = renderSphere(mask_l, sphereIn , P_l, center_l ,jac_l );

        Rect sphereInBox_r = renderSphere(mask_r, sphereIn , P_r, center_r ,jac_r);

        //update the sphereIn sphere:
        Point2d pt_l = cv_projective::reprojectPoint(sphereIn.center,P_l,Mat(),Mat(),jac_l);
        Point2d pt_r = cv_projective::reprojectPoint(sphereIn.center,P_r,Mat(),Mat(),jac_r);

        ROS_INFO_STREAM(jac_l);
        ROS_INFO_STREAM(jac_r);

        Mat jacFull(4,3,CV_64FC1);
        Mat jacFull_l = jacFull.rowRange(0,2);
        Mat jacFull_r = jacFull.rowRange(2,4);
        jac_l.copyTo(jacFull_l);
        jac_r.copyTo(jacFull_r);


        Mat blurredMask_l;
        Mat blurredMask_r;
        Mat ROI_l;
        Mat ROI_r;
        int pointOff(k_width);
        sphereInBox_l -= Point(pointOff,pointOff);
        sphereInBox_l += Size(2*pointOff,2*pointOff);

        sphereInBox_r -= Point(pointOff,pointOff);
        sphereInBox_r += Size(2*pointOff,2*pointOff);

        ROI_l = mask_l(sphereInBox_l);
        ROI_r = mask_r(sphereInBox_r);

        GaussianBlur(ROI_l,blurredMask_l,Size(pointOff*2-1,pointOff*2-1),k_var);
        GaussianBlur(ROI_r,blurredMask_r,Size(pointOff*2-1,pointOff*2-1),k_var);
        Mat segmentedImageFloat_l = segmentedImageFloat(sphereInBox_l);
        Mat segmentedImageFloat_r = segmentedImageFloat(sphereInBox_r);
        Mat weightedMask_l = blurredMask_l.mul(segmentedImageFloat_l);
        Mat weightedMask_r = blurredMask_r.mul(segmentedImageFloat_r);

        Moments mom_l = moments(weightedMask_l);
        Moments mom_r = moments(weightedMask_r);

        Mat pointOffsets(4,1,CV_64FC1);
        pointOffsets.at<double>(0) = (mom_l.m10/mom_l.m00)+sphereInBox_l.x-pt_l.x;
        pointOffsets.at<double>(1) = (mom_l.m01/mom_l.m00)+sphereInBox_l.y-pt_l.y;

        pointOffsets.at<double>(2) = (mom_r.m10/mom_r.m00)+sphereInBox_r.x-pt_r.x;
        pointOffsets.at<double>(3) = (mom_r.m01/mom_r.m00)+sphereInBox_r.y-pt_r.y;

        if(displayPause)
        {


            Mat segDisp_l(segmentedImageFloat_l*(1.0/255.0));
            Mat segDisp_r(segmentedImageFloat_r*(1.0/255.0));
            Mat weighDisp_l(weightedMask_l*(1.0/255.0));
            Mat weighDisp_r(weightedMask_r*(1.0/255.0));
            imshow("segment_r", segDisp_r);
            imshow("segment_l", segDisp_l);
            imshow("weighted_l", weighDisp_l);
            imshow("weighted_r", weighDisp_r);
            waitKey(0);
            destroyWindow("segment_l");
            destroyWindow("segment_r");
            destroyWindow("weighted_l");
            destroyWindow("weighted_r");
        }

        //transpose jacobian
        //notice the lambda of 0.4.
        ROS_INFO("The 2d image offsets are:\n");
        ROS_INFO_STREAM(pointOffsets);

        ROS_INFO("The full Jacobian is\n");
        ROS_INFO_STREAM(jacFull);
        Mat offsetList = jacFull.inv(DECOMP_SVD)*(pointOffsets*0.4);

        Point3d offsetPt(offsetList.at<double>(0),offsetList.at<double>(1),offsetList.at<double>(2));

        //update the sphereIn.
        sphereIn.center +=offsetPt; 
        ROS_INFO("The final sphere offset (in 3d space) is < %f, %f, %f >\n", offsetPt.x, offsetPt.y, offsetPt.z);

        ROS_INFO("The output sphere position is < %f, %f, %f >\n", sphereIn.center.x, sphereIn.center.y, sphereIn.center.z );

        return;
    }



    void optimizeCylinder(cylinder & cylinderIn, const Mat & segmentedImage, const Mat& P_l, const Mat&P_r , int k_width, double k_var, bool displayPause )
    {

        //create a left and right max image.
        Mat segmentedImageFloat;
        segmentedImage.convertTo(segmentedImageFloat, CV_32FC1);

        Mat mask_l = Mat::zeros(segmentedImage.size(),segmentedImageFloat.type());
        Mat mask_r = Mat::zeros(segmentedImage.size(),segmentedImageFloat.type());

        Mat jac_l,jac_r;
        Mat tips_l,tips_r;
        RotatedRect coilBox_l(renderCylinder(mask_l , cylinderIn, P_l,tips_l , jac_l));
        RotatedRect coilBox_r(renderCylinder(mask_r , cylinderIn, P_r,tips_r ,jac_r));

        Point2d center_l(coilBox_l.center.x,coilBox_l.center.y);
        Point2d center_r(coilBox_r.center.x,coilBox_r.center.y);

        int coilOff(k_width);

        Rect coilRect_l(coilBox_l.boundingRect());
        Rect coilRect_r(coilBox_r.boundingRect());

        coilRect_l -= Point(coilOff,coilOff);
        coilRect_l += Size(2*coilOff,2*coilOff);

        coilRect_r -= Point(coilOff,coilOff);
        coilRect_r += Size(2*coilOff,2*coilOff);

        Mat ROI_l = mask_l(coilRect_l);
        Mat ROI_r = mask_r(coilRect_r);

        Mat blurredMask_l,blurredMask_r;

        GaussianBlur(ROI_l,blurredMask_l,Size(coilOff*2-1,coilOff*2-1),k_var);
        GaussianBlur(ROI_r,blurredMask_r,Size(coilOff*2-1,coilOff*2-1),k_var);

        Mat segmentedImageFloat_l = segmentedImageFloat(coilRect_l);
        Mat segmentedImageFloat_r = segmentedImageFloat(coilRect_r);

        Mat weightedMask_l = blurredMask_l.mul(segmentedImageFloat_l);
        Mat weightedMask_r = blurredMask_r.mul(segmentedImageFloat_r);

        //Find the min and max of the weighted Mask.
        double min_l(0),max_l(0);

        minMaxIdx(weightedMask_l,&min_l,&max_l);

        double min_r(0),max_r(0);
        minMaxIdx(weightedMask_r,&min_r,&max_r);

        weightedMask_l *= 255/max_l;

        weightedMask_r *= 255/max_r;

        Mat weightedMaskChar_l;
        weightedMask_l.convertTo(weightedMaskChar_l,CV_8UC1);

        Mat weightedMaskChar_r;
        weightedMask_r.convertTo(weightedMaskChar_r,CV_8UC1);

        /*
         * @todo Fit the ellipse modeling to an external function
         */
        Mat binary_l;

        double thresh_l(0);
        thresh_l = threshold(weightedMaskChar_l,binary_l,127,255,THRESH_BINARY+THRESH_OTSU);

        Mat binary_r;
        double thresh_r(0);
        thresh_r = threshold(weightedMaskChar_r,binary_r,127,255,THRESH_BINARY+THRESH_OTSU);

        std::vector < std::vector < Point > > contours_l;

        ROS_INFO("why no works");

        findContours(binary_l,contours_l,noArray(),CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE,coilRect_l.tl());

        std::vector < std::vector < Point > > contours_r;
        findContours(binary_r,contours_r,noArray(),CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE,coilRect_r.tl());


        RotatedRect outputEllipse_l(minAreaRect(contours_l[0]));

        RotatedRect outputEllipse_r(minAreaRect(contours_r[0]));

        Mat  w_lx,w_rx,w_ly,w_ry;

        Scharr(weightedMask_l,w_lx,weightedMask_l.depth(),1,0);
        Scharr(weightedMask_l,w_ly,weightedMask_l.depth(),0,1);

        Scharr(weightedMask_r,w_rx,weightedMask_r.depth(),1,0);
        Scharr(weightedMask_r,w_ry,weightedMask_r.depth(),0,1);

        Mat wM_rx(w_lx.clone()), wM_ry(w_lx.clone());
        Mat wM_lx(w_lx.clone()), wM_ly(w_lx.clone());

        ROS_INFO("why no works");

        Convert the ROI to an  char

        w_rx.copyTo(wM_rx,ROI_r);
        w_ry.copyTo(wM_ry,ROI_r);


        w_lx.copyTo(wM_lx,ROI_l);
        w_ly.copyTo(wM_ly,ROI_l);


        ROS_INFO("why no works");

        Moments fx_r(moments(wM_rx));
        Moments fy_r(moments(wM_ry));

        Moments fx_l(moments(wM_lx));
        Moments fy_l(moments(wM_ly));


        double tau_l(fy_l.m10-fy_l.m00*coilBox_l.center.x-fx_l.m01+fx_l.m00*coilBox_l.center.y);

        
        double tau_r(fy_r.m10-fy_r.m00*coilBox_r.center.x-fx_r.m01+fx_r.m00*coilBox_r.center.y);


        //using the tip positions, infer a velocity in both left and right.

        double tauGain(0.0005); //start small.
        double forceGain(0.0005);

        //left image:

        Point3d omega_l(0.0,0.0,tauGain*tau_l);

        Point3d vel_l(fx_l.m00*forceGain,fy_l.m00*forceGain,0.0);

        Point3d tip0l(tips_l.at<double>(0),tips_l.at<double>(1),0.0);
        Point3d tip1l(tips_l.at<double>(3),tips_l.at<double>(4),0.0);

        Point3d centerl(tip0l*0.5+tip1l*0.5);

        Point3d vector0l(tip0l-centerl);
        Point3d vector1l(tip1l-centerl);


        Point3d vel_0l(omega_l.cross(vector0l)+vel_l);
        Point3d vel_1l(omega_l.cross(vector1l)+vel_l);


        //right image:


        Point3d omega_r(0.0,0.0,tauGain*tau_r);

        Point3d vel_r(fx_r.m00*forceGain,fy_r.m00*forceGain,0.0);

        Point3d tip0r(tips_r.at<double>(0),tips_r.at<double>(1),0.0);
        Point3d tip1r(tips_r.at<double>(3),tips_r.at<double>(4),0.0);

        Point3d centerr(tip0r*0.5+tip1r*0.5);

        Point3d vector0r(tip0r-centerr);
        Point3d vector1r(tip1r-centerr);


        Point3d vel_0r(omega_r.cross(vector0r)+vel_r);
        Point3d vel_1r(omega_r.cross(vector1r)+vel_r);


        //compare the input and output results...
        //Do this using a external function.

        //find the best fit rotated rect here:

        //Couple the two jacobian:

        Mat fullJac(8,5,CV_64FC1);

        jac_l.copyTo(fullJac.rowRange(0,4));
        jac_r.copyTo(fullJac.rowRange(4,8));

        Mat imageOffset(8,1,CV_64FC1);

        imageOffset.at<double>(0) = vel_0l.x;
        imageOffset.at<double>(1) = vel_0l.y;
        imageOffset.at<double>(2) = vel_1l.x;
        imageOffset.at<double>(3) = vel_1l.y;

        imageOffset.at<double>(5) = vel_0r.x;
        imageOffset.at<double>(6) = vel_0r.y;
        imageOffset.at<double>(7) = vel_1r.x;
        imageOffset.at<double>(8) = vel_1r.y;

        Mat offsetVector(fullJac.inv(DECOMP_SVD)*imageOffset);


        Point3d cylinder_offset(offsetVector.at<double>(0),offsetVector.at<double>(1),offsetVector.at<double>(2));
        double thetaOffset(offsetVector.at<double>(3));
        double phiOffset(offsetVector.at<double>(4));
      if(displayPause)
      {


          ROS_INFO("The offset of the left points are  < %f , %f  > and  < %f , %f  >\n", vel_0l.x,vel_0l.y, vel_1l.x, vel_1l.y);
          ROS_INFO("The offset of the right points are  < %f , %f  > and  < %f , %f  >\n", vel_0r.x,vel_0r.y, vel_1r.x, vel_1r.y);


          ROS_INFO("The Cylinder update is <%f m, %f m, %f m, %f rads, %f rads > \n", cylinder_offset.x,cylinder_offset.z, cylinder_offset.z, thetaOffset,phiOffset);


          Mat segDisp_l(segmentedImageFloat_l*(1.0/255.0));
            Mat segDisp_r(segmentedImageFloat_r*(1.0/255.0));
            Mat weighDisp_l(weightedMask_l*(1.0/255.0));
            Mat weighDisp_r(weightedMask_r*(1.0/255.0));
            imshow("segment_r", segDisp_r);
            imshow("segment_l", segDisp_l);
            imshow("weighted_l", weighDisp_l);
            imshow("weighted_r", weighDisp_r);
            waitKey(0);
            destroyWindow("segment_l");
            destroyWindow("segment_r");
            destroyWindow("weighted_l");
            destroyWindow("weighted_r");
      }

      
      //look at the twist from the original image to the new image.
      //which eigenvector is bigger?
      //find the largest eigenvector.

      //largest left eigen:
      //leftRow = 
      cylinderIn.center +=cylinder_offset;
      cylinderIn.theta += thetaOffset;
      cylinderIn.phi += phiOffset;

    }

cv::Point3d computeNormalFromSpherical(double theta, double phi, OutputArray jac)
{
    Point3d output;

    output.x = sin(phi)*cos(theta);
    output.y = sin(phi)*sin(theta);
    output.z = cos(phi);

    if(jac.needed())
    {
        jac.create(3,2,CV_64FC1);
        Mat jac_ = jac.getMat();
        jac_.setTo(0);
        jac_.at<double>(0,0) = -sin(phi)*sin(theta);
        jac_.at<double>(0,1) = cos(phi)*cos(theta);
        jac_.at<double>(1,0) = sin(phi)*cos(theta);
        jac_.at<double>(1,1) = cos(phi)*sin(theta);
        jac_.at<double>(2,0) = 0.0;
        jac_.at<double>(2,1) = -sin(phi);
    }
    return output;

}


    cylinder baseCylinder()
    {
        cylinder newCylinder;

        newCylinder.center = Point3d(0.0,0.0,0.0);
        newCylinder.theta = 0.0;
        newCylinder.phi =0.0;
        newCylinder.height = 0.0128; //(in meters)
        newCylinder.radius = 0.0025;

        return newCylinder; 
    }



};
