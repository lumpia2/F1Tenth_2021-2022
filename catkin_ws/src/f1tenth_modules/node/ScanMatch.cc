/**
 * @file scan_match.cpp
 * @author Justin Cacal (jcc159@pitt.edu)
 * @brief This file follows lab 5 of the F1Tenth lab modules (https://f1tenth.org/learn.html)
 * @version 0.1
 * @date 2022-09-09
 *
 * @copyright Copyright (c) 2022
 *
 */

 class ScanMatch 
 {
    private:
        ros::NodeHandle n;
        ros::Publisher pub;
        ros::Subscriber sub;

        lidarIntrinsics lidarData;

    public:
        ScanMatch() : n(ros::NodeHandle("~"))
        {
            lidarData = getLidarInfoFromTopic(n, "/scan")
        }
    /*
        ICP Algorithm

        Input:
        Reference scan (y_(t-1))
        Current scan (y_t)
        Initial Guess R_transformation

        Output:
        Transformation (converged solution)

        Iterative Search for R:
        1. Make guess of R (current guess)
        2. For each point in new scan (t=k+1)
           find closest point in previous set (t=k)
        3. Make better guess of R
        4. Set next guess to better guess of R
           and repeat 2-4 until convergence

        Choosing Best Guess R:
        Point-to-Line Method
    */
 }