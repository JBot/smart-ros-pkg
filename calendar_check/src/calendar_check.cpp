/* ROS includes */
#include <ros/ros.h>
#include <std_msgs/Int32.h>

/* C includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

/* C++ includes */
#include <iostream>
#include <string>
#include <list>


class calendarCheck {
    public:
        calendarCheck();

        //ros::Subscriber pose_sub_;
        ros::Publisher command_pub;

        void check_calendar(void);

    private:

	ros::NodeHandle nh;

};

/* Constructor */
calendarCheck::calendarCheck()
{
    command_pub = nh.advertise < std_msgs::Int32 > ("/calendar/command", 3);
}

void calendarCheck::check_calendar(void)
{
	/* Open the calendar */
        FILE * f = popen( "google calendar --cal=YOUR-AGENDA-HERE today", "r" );
        if ( f == 0 ) {
                fprintf( stderr, "Could not execute\n" );
                return;
        }
        const int BUFSIZE = 256;
        char buff[ BUFSIZE ];
        int i = 0;

        int hour_now, minute_now, hour_cal, minute_cal;
        unsigned pos;

        time_t rawtime;
        struct tm * timeinfo;

	/* Ask for the time NOW */
        time ( &rawtime );
        timeinfo = localtime ( &rawtime );
        //printf ( "The current date/time is: %s", asctime (timeinfo) );

	/* Only takes the hours and minutes */
        std::string str_date = asctime (timeinfo);
        pos = str_date.find(" ");
        str_date = str_date.substr(pos+1);
        pos = str_date.find(" ");
        str_date = str_date.substr(pos+1);
        pos = str_date.find(" ");
        str_date = str_date.substr(pos+1);
        hour_now = atoi((str_date.substr(0, 2)).c_str());
        minute_now = atoi((str_date.substr(3, 2)).c_str());

        //printf ( "Heure : %d::%d\n", hour_now, minute_now );
	
	/* Convert it into minutes only */
        minute_now = minute_now + hour_now * 60;


	/* Read each line in the calendar answer */
        while( fgets( buff, sizeof( buff ),  f ) ) {
                /* Remove line 1 and 2 */
		if(i<2) {
                }
                else {
			/* Look for the beginning time */
                        std::string my_string = buff;
                        pos = my_string.find(",");
                        std::string str3 = my_string.substr(pos);

                        pos = str3.find(" ");
                        str3 =  str3.substr(pos+1);
                        pos = str3.find(" ");
                        str3 =  str3.substr(pos+1);

                        hour_cal = atoi((str3.substr(0, 2)).c_str());
                        minute_cal = atoi((str3.substr(3, 2)).c_str());

                        //std::cout << hour_cal << "::" << minute_cal << std::endl;

			/* Convert it into minutes only */
                        minute_cal = minute_cal + hour_cal * 60;

                        if((minute_cal < minute_now) || ((minute_cal-minute_now) > 5)) {
                                //std::cout << "Event already started or not in less than 5 minutes" << std::endl;
                        }
                        else {
				/* Check if it is a command or something to remind */
				if (my_string.compare(0,7,"command") == 0) {
					/* This is a command */
					std_msgs::Int32 my_command;
					my_command.data = atoi((my_string.substr(8, 2)).c_str());
					std::cout << "Commande numéro :" << my_command.data << std::endl;
				}
				else { 
					/* This is something to say */
					char* nl = strrchr(buff, '\r');
					if (nl) *nl = '\0';
					nl = strrchr(buff, '\n');
					if (nl) *nl = '\0';
					char my_buff[256];
					//cout << buff;

					sprintf(my_buff, "espeak -g 1 -v us-mbrola-1 \" In less than 5 minutes \"");
					std::cout << my_buff << std::endl;
					system(my_buff);
					sprintf(my_buff, "espeak -g 1 -v us-mbrola-1 \" %s \"", buff);
					std::cout << my_buff << std::endl;
					usleep(1000000);
					system(my_buff);
				}
			}
		}
		i++;
	}
	pclose( f );



}


int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line. For programmatic
	 * remappings you can use a different version of init() which takes remappings
	 * directly, but for most command-line programs, passing argc and argv is the easiest
	 * way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "calendar_check");
	calendarCheck calendarcheck;
	// Refresh rate
	ros::Rate loop_rate(0.003333333); /* 5 min */
	while (ros::ok()) {
		calendarcheck.check_calendar();
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::Duration(2.0).sleep();

	ros::shutdown();
}

