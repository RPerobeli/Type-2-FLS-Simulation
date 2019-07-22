#include "mission.h"

mission::mission(char end[])
{
    waypointTolerance = 1; // [m]
    readJson(end);
}

mission::~mission(){

}

void mission::setWpTolerance(int wpTol)
{
    waypointTolerance = wpTol;
}

void mission::readJson(char path[])
{
    string a;
    ifstream file(path);

    if (file.is_open())
    {
        file >> a;
    }
    else
        cout << "Unable to open file.\n";

    j.Parse(a.c_str());

    numWp = j["waypoints"].Size();

    wp = new mavros_msgs::Waypoint[numWp];

    const rapidjson::Value& w = j["waypoints"];
    for (int i = 0; i < numWp; i++)
    {
        //MAV_FRAME_GLOBAL_RELATIVE_ALT
        //Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position.
        //First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location.
        wp[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        wp[i].autocontinue = true;

        wp[i].x_lat = w[i]["latitude"].GetDouble();
        wp[i].y_long = w[i]["longitude"].GetDouble();
        wp[i].z_alt = w[i]["altitude"].GetDouble();

        if (!strcmp(w[i]["type"].GetString(), "WAYPOINT"))
        {
            //MAV_CMD_NAV_WAYPOINT	Navigate to MISSION.
            wp[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;

            //Mission Param #1	Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)
            wp[i].param1 = w[i]["duration"].GetInt();

            //Mission Param #2	Acceptance radius in meters (if the sphere with this radius is hit, the MISSION counts as reached)
            wp[i].param2 = waypointTolerance;

            //Mission Param #3	0 to pass through the WP, if > 0 radius in meters to pass by WP.
            //Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
            wp[i].param3 = 20;

            //Mission Param #4	Desired yaw angle at MISSION (rotary wing)
            wp[i].param4 = 0;
        }

        else if (!strcmp(w[i]["type"].GetString(), "DECOLAGEM"))
        {
            //MAV_CMD_NAV_TAKEOFF	Takeoff from ground / hand
            wp[i].command = mavros_msgs::CommandCode::NAV_TAKEOFF;

            //Mission Param #1	Minimum pitch (if airspeed sensor present), desired pitch without sensor
            wp[i].param1 = 0;

            //Mission Param #2	Empty
            wp[i].param2 = 0;

            //Mission Param #3	Empty
            wp[i].param3 = 0;

            //Mission Param #4	Yaw angle (if magnetometer present), ignored without magnetometer
            wp[i].param4 = 0;
        }

        else if (!strcmp(w[i]["type"].GetString(), "POUSO"))
        {
            //MAV_CMD_NAV_LAND	Land at location
            wp[i].command = mavros_msgs::CommandCode::NAV_LAND;

            //Mission Param #1	Abort Alt
            // Altitude para ir ao abortar o pouso. ONLY FIXED WING.
            // http://ardupilot.org/plane/docs/common-mavlink-mission-command-messages-mav_cmd.html#id13
            wp[i].param1 = 0;

            //Mission Param #2	Empty
            wp[i].param2 = 0;

            //Mission Param #3	Empty
            wp[i].param3 = 0;

            //Mission Param #4	Desired yaw angle
            wp[i].param4 = 0;
        }

        // Source:
        // http://docs.ros.org/api/mavros_msgs/html/msg/Waypoint.html
        // http://docs.ros.org/api/mavros_msgs/html/msg/CommandCode.html
        // https://pixhawk.ethz.ch/mavlink
    }
}

void mission::printJson()
{
    cout << "Waypoints" << endl;
    for (int i = 0; i < numWp; i++)
    {
        cout << "  idx: " << i + 1 << endl;
        cout << "    Type: " << wp[i].command << endl;
        cout << "    Latitude: " << wp[i].x_lat << endl;
        cout << "    Longitude: " << wp[i].y_long << endl;
        cout << "    Altitude: " << wp[i].z_alt << endl;
        cout << "    Param3: " << wp[i].param3 << endl;
    }
    cout << endl;
}

int mission::getJsonSize()const
{
    return numWp;
}

mavros_msgs::Waypoint* mission::getJsonWp()const
{
    mavros_msgs::Waypoint* temp;
    temp = new mavros_msgs::Waypoint[numWp];
    for (int i = 0; i < numWp; i++)
    {
        temp[i].frame = wp[i].frame;
        temp[i].x_lat = wp[i].x_lat;
        temp[i].y_long = wp[i].y_long;
        temp[i].z_alt = wp[i].z_alt;
        temp[i].command = wp[i].command;
        temp[i].param1 = wp[i].param1;
        temp[i].param2 = wp[i].param2;
        temp[i].param3 = wp[i].param3;
        temp[i].param4 = wp[i].param4;
        temp[i].autocontinue = wp[i].autocontinue;
    }
    return temp;
}

mavros_msgs::Waypoint mission::getJsonWp(int index)const
{
    return wp[index];
}
