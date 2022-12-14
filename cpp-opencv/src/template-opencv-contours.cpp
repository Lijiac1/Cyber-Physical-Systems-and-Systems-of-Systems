/*
 * Copyright (C) 2020  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Include the single-file, header-only middleware libcluon to create high-performance microservices
#include "cluon-complete.hpp"
// Include the OpenDLV Standard Message Set that contains messages that are usually exchanged for automotive or robotic applications 
#include "opendlv-standard-message-set.hpp"

// Include the GUI and image processing header files from OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
void getContours(cv::Mat img);


int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    // Parse the command line parameters as we require the user to specify some mandatory information on startup.
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ||
         (0 == commandlineArguments.count("name")) ||
         (0 == commandlineArguments.count("width")) ||
         (0 == commandlineArguments.count("height")) ) {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=253 --name=img --width=640 --height=480 --verbose" << std::endl;
    }
    else {
        // Extract the values from the command line parameters
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid()) {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Interface to a running OpenDaVINCI session where network messages are exchanged.
            // The instance od4 allows you to send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            opendlv::proxy::GroundSteeringRequest gsr;
            std::mutex gsrMutex;
            auto onGroundSteeringRequest = [&gsr, &gsrMutex](cluon::data::Envelope &&env){
                // The envelope data structure provide further details, such as sampleTimePoint as shown in this test case:
                // https://github.com/chrberger/libcluon/blob/master/libcluon/testsuites/TestEnvelopeConverter.cpp#L31-L40
                std::lock_guard<std::mutex> lck(gsrMutex);
                gsr = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(env));
                std::cout << "lambda: groundSteering = " << gsr.groundSteering() << std::endl;
            };

            od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);

            // Endless loop; end the program by pressing Ctrl-C.
            // counter for frame
            int counter = 0;
            while (od4.isRunning()) {
                // when the fram comes, records the time
                //Calculate the fps
                int64_t mSeconds = cluon::time::toMicroseconds(cluon::time::now());
                double tick = static_cast<double>(cv::getTickCount());

                // OpenCV data structure to hold an image.
                // and declear the croped img
                cv::Mat img, imgCrop;
                // resize the image;
                cv::Rect roi(0,HEIGHT*0.535,WIDTH,HEIGHT*0.465);
                counter++;
                // Wait for a notification of a new frame.
                sharedMemory->wait();

                // Lock the shared memory.
                sharedMemory->lock();
                {
                    // Copy the pixels from the shared memory into our own data structure.
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                    img = wrapped.clone();
                    imgCrop = img(roi);
                    
                }
                // TODO: Here, you can add some code to check the sampleTimePoint when the current frame was captured.
                sharedMemory->unlock();
                
                //get the time in micro seconds from cluon lib
                
                //convert it to string
                std::string utcTime_M = std::to_string(mSeconds-7200000000);
                //get time from 
                std::time_t seconds = std::time(nullptr);
                //get time in seconds
                std::tm* t = gmtime(&seconds);
                char ts[50];
                //convert it to required type
                std::strftime(ts,50, "%Y-%m-%dT%H:%M:%SZ;",t);
                std::string name = "Jiacheng Li ";
                std::string timeStamp = ts;
                std::string composed = "Now:" + timeStamp + " ts:"+utcTime_M+"; "+name;
                
                
                // t is the counts of the event frequency
                tick = ((double)cv::getTickCount()-tick)/cv::getTickFrequency();
                double fps = 1.0/tick;
                //get the end time
                int64_t end = cluon::time::toMicroseconds(cluon::time::now());
                //get how much time spent for each fram
                int64_t gap = end - mSeconds;
                //double frameRate = 1000000.0/(double)gap;
                //char arry to put the fps
                char fgap[100];
                char ftick[100];
                //c ways to keep two decimal digits
                // formating the data
                std::string count = std::to_string(counter);
                std::sprintf(fgap,"%ld",gap);
                std::sprintf(ftick,"%.2f",fps);
                std::string fpsGap(" GAP: ");
                std::string fpsTick(" FPS: ");
                fpsTick += ftick;
                fpsGap += fgap;
                composed += count;
                // TODO: Do something with the frame.
                // Example: Draw a red rectangle and display image.
                cv::rectangle(img, cv::Point(50, 50), cv::Point(100, 100), cv::Scalar(0,0,255));
                // add time to each fram
                cv::putText(img,composed, cv::Point(22,22), cv::FONT_HERSHEY_PLAIN ,1, cv::Scalar(255,255,255));
                cv::putText(img,fpsGap, cv::Point(22,33), cv::FONT_HERSHEY_PLAIN ,1, cv::Scalar(255,255,255));
                cv::putText(img,fpsTick, cv::Point(22,44), cv::FONT_HERSHEY_PLAIN ,1, cv::Scalar(255,255,255));
                // If you want to access the latest received ground steering, don't forget to lock the mutex:
                {
                    std::lock_guard<std::mutex> lck(gsrMutex);
                    std::cout << "main: groundSteering = " << gsr.groundSteering() << std::endl;
                }

                // Display image on your screen.
                if (VERBOSE) {
                    getContours(imgCrop);
                    cv::imshow(sharedMemory->name().c_str(), img);
                    //cv::imshow("imgCrop",imgCrop);
                    
                    cv::waitKey(1);
                }
            }
        }
        retCode = 0;
    }
    return retCode;
}

void getContours(cv::Mat img){
    // mat to generate the contours
    cv::Mat imgGray, imgBlur, imgCanny, imgDil, imgCopy;
    //copy the img
    img.copyTo(imgCopy);
    // convert the img to Gray
    cv::cvtColor(img, imgGray, COLOR_BGR2GRAY);
    // add blur for the converted img
    cv::GaussianBlur(imgGray, imgBlur, Size(3,3),3,0);
    // canny the img
    cv::Canny(imgBlur, imgCanny, 25, 75);
    // dilate the img
    cv::Mat kernel = cv::getStructuringElement(MORPH_RECT, Size(3,3));
    cv::dilate(imgCanny, imgDil, kernel);

    // find out the contours that we need 
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(imgDil, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    //detect the polygon
    std::vector<std::vector<cv::Point>> conPloy(contours.size());

    for (int i = 0; i < contours.size(); i++){
        
        int area = contourArea(contours[i]);
        // sorting the contours
        if(area < 800 && area > 10 ){
            // detect the polygon
            float peri = arcLength(contours[i], true);
            cv::approxPolyDP(contours[i], conPloy[i], 0.02*peri, true);
            // sorting the needded polygon
            if(conPloy[i].size() <= 9){
                cv::drawContours(imgCopy, conPloy, i, Scalar(255,0,0), 2);

            }
            
        };


    }

    cv::imshow("contours",imgDil);
    cv::imshow("imgComposed",imgCopy);

};