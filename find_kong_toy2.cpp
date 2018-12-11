//get images from topic "/usb_cam/image_raw"; remap, as desired;
//search for kong dog toy pixels;
// convert (sufficiently) toy pixels to white, all other pixels black
// compute centroid of kong dog toy pixels and display as a blue square
// publish result of processed image on topic "/image_converter/output_video"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <xform_utils/xform_utils.h>
#include <Eigen/Eigen> //for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <vector>
using namespace std;

static const std::string OPENCV_WINDOW = "Open-CV display window";
using namespace std;

//height of kong toy, lying on its side, at desired grasp location:
//const double OBJECT_HEIGHT=0.018; // hard-coded top surface of object relative to world frame
const double OBJECT_HEIGHT= 0.0145; // hard-coded top surface of object relative to world
const double GRASP_OFFSET_ALONG_AXIS = 0.012; //guess at 2cm; TUNE THIS

Eigen::Matrix2d g_R_camera;
    Eigen::Vector2d  col1(0.023050, -0.999734);
    Eigen::Vector2d  col2(-0.999734,-0.023050);
    Eigen::Vector2d xy_robot_derived, cam_coords_ij, cam_coords_xy;
/*t =

   0.91489
   0.51511
*/
Eigen::Vector2d g_t_camera(0.91489,0.51511);

const double pix_per_m= 1122.0;
const double PIX_LEN_OFFSET = GRASP_OFFSET_ALONG_AXIS*pix_per_m;


class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher block_pose_publisher_; // = n.advertise<std_msgs::Float64>("topic1", 1);
    geometry_msgs::PoseStamped block_pose_;
    XformUtils xformUtils;

public:

    ImageConverter(ros::NodeHandle &nodehandle)
    : it_(nh_) {
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
                &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        block_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("object_pose", 1, true); 
        block_pose_.header.frame_id = "world"; //specify the  block pose in world coords
        block_pose_.pose.position.z = OBJECT_HEIGHT;
        block_pose_.pose.position.x = 0.5; //not true, but legal
        block_pose_.pose.position.y = 0.0; //not true, but legal
        
        // need camera info to fill in x,y,and orientation x,y,z,w
        //geometry_msgs::Quaternion quat_est
        //quat_est = xformUtils.convertPlanarPsi2Quaternion(yaw_est);
        block_pose_.pose.orientation = xformUtils.convertPlanarPsi2Quaternion(0); //not true, but legal
        
        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    //image comes in as a ROS message, but gets converted to an OpenCV type
    void imageCb(const sensor_msgs::ImageConstPtr& msg); 
    
}; //end of class definition

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr cv_ptr; //OpenCV data type
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // look for red pixels; turn all other pixels black, and turn red pixels white
        int npix = 0; //count the red pixels
        int isum = 0; //accumulate the column values of red pixels
        int jsum = 0; //accumulate the row values of red pixels
        int redval, blueval, greenval, testval;
        cv::Vec3b rgbpix; // OpenCV representation of an RGB pixel
        //comb through all pixels (j,i)= (row,col)
        vector<double> i_vals_object,j_vals_object;
  
        for (int i = 0; i < cv_ptr->image.cols; i++) {
            for (int j = 0; j < cv_ptr->image.rows; j++) {
                rgbpix = cv_ptr->image.at<cv::Vec3b>(j, i); //extract an RGB pixel
                //examine intensity of R, G and B components (0 to 255)
                redval = rgbpix[2] + 1; //add 1, to avoid divide by zero
                blueval = rgbpix[0] + 1;
                greenval = rgbpix[1] + 1;
                //look for red values that are large compared to blue+green
                testval = 255 - blueval;
                int net_val = redval+blueval+greenval;
                //if pixel is the color of kong dog toy, paint it white:
                //if (testval <= 95) {
            //xxx wsn modification 12/8/18: can use with bottlecap or Kong toy
            //merely a gray-scale threshold
                if (net_val >= 700) {
                    cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 255;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 255;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 255;
                    npix++; //note that found another red pixel
                    isum += i; //accumulate row and col index vals
                    jsum += j;
                    i_vals_object.push_back( (double) i);
                    j_vals_object.push_back( (double) j);
                } else { //else paint it black
                    cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 0;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 0;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 0;
                }
            }
        }
        //cout << "npix: " << npix << endl;
        //paint in a blue square at the centroid:
        int half_box = 5; // choose size of box to paint
        int i_centroid, j_centroid;
        double x_centroid, y_centroid;
        if (npix > 0) {
            i_centroid = isum / npix; // average value of u component of red pixels
            j_centroid = jsum / npix; // avg v component
            x_centroid = ((double) isum)/((double) npix); //floating-pt version
            y_centroid = ((double) jsum)/((double) npix);
            //ROS_INFO("u_avg: %f; v_avg: %f",x_centroid,y_centroid);
            //cout << "i_avg: " << i_centroid << endl; //i,j centroid of red pixels
            //cout << "j_avg: " << j_centroid << endl;
            for (int i_box = i_centroid - half_box; i_box <= i_centroid + half_box; i_box++) {
                for (int j_box = j_centroid - half_box; j_box <= j_centroid + half_box; j_box++) {
                    //make sure indices fit within the image 
                    if ((i_box >= 0)&&(j_box >= 0)&&(i_box < cv_ptr->image.cols)&&(j_box < cv_ptr->image.rows)) {
                        cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[0] = 255; //(255,0,0) is pure blue
                        cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[1] = 0;
                        cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[2] = 0;
                    }
                }
            }

        }

        //wsn 12/8/18...find major axis

        Eigen::MatrixXd ij_vecs;  //matrix(i,j)
        ij_vecs.resize(2,npix);
        for (int j=0;j<npix;j++) {
          ij_vecs(0,j) = i_vals_object[j]-x_centroid;
          ij_vecs(1,j) = j_vals_object[j]-y_centroid;
	}
       //compute the covariance matrix w/rt x,y,z:

    	Eigen::Matrix2d CoVar;
    	CoVar = ij_vecs * (ij_vecs.transpose()); //2xN matrix times 2x3 matrix is 2x2
        Eigen::EigenSolver<Eigen::Matrix2d> es2d(CoVar);  //construct an Eigen solver
        Eigen::Vector2d evals;
        evals = es2d.eigenvalues().real(); // grab just the real parts
        int major_axis_index=0;
        int minor_axis_index=1;
        if (evals(1)>evals(0)) {
           major_axis_index=1;
           minor_axis_index=0;
        }

        Eigen::Vector2d major_axis_vec = es2d.eigenvectors().col(major_axis_index).real();
        Eigen::Vector2d minor_axis_vec = es2d.eigenvectors().col(minor_axis_index).real();
        //now need to find the "fat" end and translate grasp site towards tip
        //transform the ij_vecs into principal-axes frame;

        Eigen::Vector2d ij_centroid,ij_plus_grasp,ij_minus_grasp;
        ij_centroid<<x_centroid,y_centroid;
        Eigen::Matrix2d R_axes;
        R_axes.col(0) = major_axis_vec;
        R_axes.col(1) = minor_axis_vec;
        Eigen::MatrixXd transformed_ij_vecs;  //matrix(i,j)
        transformed_ij_vecs.resize(2,npix);
        transformed_ij_vecs = R_axes.transpose()*ij_vecs;

        //now compute centroid of a band of pixels along major axis, +/- a few pixels in y dir
        double x_cum=0;
        double y_cum=0;
        double n_cum=0;
        int i_centroid2,j_centroid2;
        for (int ipix=0;ipix<npix;ipix++) {
          if (fabs(transformed_ij_vecs(1,ipix))<10) { //is pixel within 10 pixels of major axis?)
           x_cum+= transformed_ij_vecs(0,ipix);
           y_cum+= transformed_ij_vecs(1,ipix);
           n_cum+=1.0;
          }
        }
        double x_centroid_in_principal_coords = x_cum/n_cum;
        double y_centroid_in_principal_coords = y_cum/n_cum;
 
        Eigen::Vector2d xy_centroid2_in_principal_coords;
        xy_centroid2_in_principal_coords(0) = x_centroid_in_principal_coords;
        xy_centroid2_in_principal_coords(1) =y_centroid_in_principal_coords;
        //transform back to camera coords:
        Eigen::Vector2d xy_centroid2_wrt_cam = R_axes*xy_centroid2_in_principal_coords;
        i_centroid2 = (int) (xy_centroid2_wrt_cam(0)) + i_centroid;
        j_centroid2 = (int) (xy_centroid2_wrt_cam(1)) + j_centroid;


        //SHOULD have x_centroid_in_principal_coords >0 if it is shifted towards narrow end
        //ROS_INFO("x_centroid_in_principal_coords= %f",x_centroid_in_principal_coords);
        if (x_centroid_in_principal_coords<0) major_axis_vec = -1.0*major_axis_vec;




        ij_plus_grasp = ij_centroid + PIX_LEN_OFFSET* major_axis_vec;
        ij_minus_grasp = ij_centroid - PIX_LEN_OFFSET* major_axis_vec;

        i_centroid = (int) (ij_plus_grasp(0)); //convert back to integer pixels
        j_centroid = (int) (ij_plus_grasp(1)); //convert back to integer pixels

        //paint the candidate grasp sites:
            for (int i_box = i_centroid - half_box; i_box <= i_centroid + half_box; i_box++) {
                for (int j_box = j_centroid - half_box; j_box <= j_centroid + half_box; j_box++) {
                    //make sure indices fit within the image 
                    if ((i_box >= 0)&&(j_box >= 0)&&(i_box < cv_ptr->image.cols)&&(j_box < cv_ptr->image.rows)) {
                        cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[0] = 0; //(255,0,0) is pure blue
                        cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[1] = 0;
                        cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[2] = 255;
                    }
                }
            }




        // Update GUI Window; this will display processed images on the open-cv viewer.
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3); //need waitKey call to update OpenCV image window

        // Also, publish the processed image as a ROS message on a ROS topic
        // can view this stream in ROS with: 
        //rosrun image_view image_view image:=/image_converter/output_video
        image_pub_.publish(cv_ptr->toImageMsg());
        cam_coords_ij(0) = ij_plus_grasp(0);  //try ij_plus_grasp(0)
        cam_coords_ij(1) = ij_plus_grasp(1);  //try ij_plus_grasp(1)
	cam_coords_xy = cam_coords_ij/pix_per_m;
        xy_robot_derived = g_R_camera*(cam_coords_xy-g_t_camera);

        
        block_pose_.pose.position.x = xy_robot_derived(0) + 0.0165; //not true, but legal
        block_pose_.pose.position.y = xy_robot_derived(1); //not true, but legal
        double theta=0;
        
        // need camera info to fill in x,y,and orientation x,y,z,w
        //geometry_msgs::Quaternion quat_est
        //quat_est = xformUtils.convertPlanarPsi2Quaternion(yaw_est);
        block_pose_.pose.orientation = xformUtils.convertPlanarPsi2Quaternion(theta); //not true, but legal
        block_pose_publisher_.publish(block_pose_);
    }

int main(int argc, char** argv) {
    ros::init(argc, argv, "red_pixel_finder");
    ros::NodeHandle n; //     
   /*R =

   0.023050  -0.999734
  -0.999734  -0.023050

t =

   0.91489
   0.51511
*/


    g_R_camera.col(0) = col1;
    g_R_camera.col(1) = col2;    
    //g_t_camera << 
    ImageConverter ic(n); // instantiate object of class ImageConverter
    //cout << "enter red ratio threshold: (e.g. 10) ";
    //cin >> g_redratio;
    //choose a threshold to define what is "red" enough
    ros::Duration timer(0.1);
    double x, y, z;
    while (ros::ok()) {
        ros::spinOnce();
        timer.sleep();
    }
    return 0;
}
