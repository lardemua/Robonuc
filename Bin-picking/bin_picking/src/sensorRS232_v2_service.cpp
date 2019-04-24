
/**
*      @file  
*      @brief  Program to get average distance (adapted from Joana Mota 2018)
*
* Descrição mais detalhada do ficheiros
*
*     @author  Tiago Tavares, tiagoatavares@ua.pt
*
*   @internal
*     Created  24-April-2019
*     Company  University of Aveiro
*   Copyright  Copyright (c) 2019, Tiago Tavares
*
***************************************************
*/

#include "../../bin_picking/include/bin_picking/sensorRS232.h"

#include "ros/ros.h"
#include "bin_picking/get_laser_average.h"

#include <unistd.h>

using namespace std;

char distance_laser[10];
bool service_request = false;

/**
 * @brief  Reads from port until a specific char. 
 * 
 * This functions reads the values received until it finds a paragraph, to storage the intire distance value.
 *
 * @param  fd - The file descriptor
 * @return none
 */
int ReadPortUntilChar(int fd)
{
    char ch;
    int n;
    do
    {
        n = read(fd, &ch, 1);
        if (n == -1 || n == 0)
            continue; //perror("Err:");
        sprintf(distance_laser, "%s%c", distance_laser, ch);
    } while (ch != '\n'); //Reads until a paragraph is found
    return 0;
}

bool service_cb(bin_picking::get_laser_average::Request &req,
                bin_picking::get_laser_average::Response &res)
{
    res.my_response = req.my_request;
    ROS_INFO("SERVICE CALLED ! sending back response: [%ld]", (long int)res.my_response);
    service_request = true;
    return true;
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "bin_picking_sensorRS232_service");

    ros::NodeHandle nh;
    ros::Publisher pub_rs232 = nh.advertise<std_msgs::Float32>("/output_laser_sensor_average", 1000);
    // ros::Rate loop_rate(15);

    ros::ServiceServer service = nh.advertiseService("get_laser_average", service_cb);
    service_request = false;

    int fd;
    
    
    ros::Rate r(100);
    while (nh.ok())
    {
        fd = OpenPort("/dev/ttyACM0", NULL); //need to change ....
        ros::spinOnce();
        
        if (fd == -1)
        {
            cout << "Error. Could not open LASER port" << endl;
            //fd = OpenPort("/dev/ttyACM0", NULL); //need to change ....
            usleep(1000000);
        }
        else
        {
            if (service_request)
            {
                float counter = 0.0;
                vector<float> readings;

                while (counter < 11)
                {

                    std_msgs::Float32 dist;
                    ReadPortUntilChar(fd); //Reads the distance given by the Arduino UNO
                    dist.data = atof(distance_laser);
                    if (dist.data > 100.0 && dist.data < 600.0)
                    {
                        cout << "Distance=" << dist.data << endl;

                        if (counter > 0)
                        {
                            readings.push_back(dist.data);
                        }
                        counter++;
                    }
                    distance_laser[0] = '\0';
                }

                float average = accumulate(readings.begin(), readings.end(), 0.0) / readings.size();

                cout << "The size is: " << readings.size() << endl;
                cout << "The average is: " << average << endl;
                std_msgs::Float32 dist_average;
                dist_average.data = average;

                pub_rs232.publish(dist_average);
                service_request=false;
            }
        }
        
        close(fd);
    }

    

    

    return 0;

    // fd=OpenPort("/dev/ttyACM0", NULL); //need to change ....

    // while (ros::ok() & fd == -1)
    // {
    //     cout << "Error. Could not open port" << endl ;
    //     // exit(1);
    // }

    // char distance_laser[10];
    // float counter = 0.0;

    // while (ros::ok() && counter < 11)
    // {
    //     // cout << "aqui" << endl;
    //     std_msgs::Float32 dist;
    //     ReadPortUntilChar(fd); //Reads the distance given by the Arduino UNO
    //     dist.data = atof(distance_laser);
    //     if (dist.data > 100.0 && dist.data < 600.0)
    //     {
    //         cout << "Distance=" << dist.data << endl;

    //         if (counter > 0)
    //         {
    //             readings.push_back(dist.data);
    //         }
    //         counter++;
    //     }
    //     distance_laser[0] = '\0';
    //     // loop_rate.sleep();
    // }

    // float average = accumulate(readings.begin(), readings.end(), 0.0) / readings.size();

    // cout << "The size is: " << readings.size() << endl;
    // cout << "The average is: " << average << endl;
    // std_msgs::Float32 dist_average;
    // dist_average.data = average;
    // close(fd);
    // while (ros::ok())
    // {
    //     pub_rs232.publish(dist_average);
    //     // cout << "Output Laser just published" << endl;
    // }

    // return 0;
}