/* ----------------------------------------------------------------------------

 * Author: Abhishek Goudar.

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#include <ra_sam/ra_sam.h>


int main(int argc, char** argv)
{
    if(argc < 2)
    {
        printf("No config file provided.\n");
    }
    else
    {
        char* sam_config = realpath(argv[1], NULL);
        char* robot_config = realpath(argv[2], NULL);
        // Check if config file exists  
        if(sam_config != NULL && robot_config != NULL)
        {
            printf("SAM Config file provided:[%s]\n", sam_config);
            printf("Robot Config file provided:[%s]\n", robot_config);
            ros::init(argc, argv, "RaSam");
            sam::RaSam sam(sam_config, robot_config);
            sam.runEstimator();  
        } 
        else
        {
            if(sam_config == NULL)
                printf("File:[%s] not found,\n", sam_config);
            if(robot_config == NULL)
                printf("File:[%s] not found,\n", robot_config);
        }
        free(sam_config);
        free(robot_config);
    }

    return 0;
}