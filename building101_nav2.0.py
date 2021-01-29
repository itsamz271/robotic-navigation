#!/usr/bin/env python  

import sqlite3
import math
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from turtlesim.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
import os.path

############################################
#######  CONNECTING TO DATABASE  ###########
############################################
database_filename = 'building101.db'

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
db_path = os.path.join(BASE_DIR, database_filename)



conn = sqlite3.connect(db_path)

c = conn.cursor()

#####Creating a table called 'locations'#####

#c.execute("""CREATE TABLE locations (
#    name text,
#    x_coordinate real,
#    y_coordinate real,
#    orientation_x real, 
#    orientation_y real, 
#    orientation_z real,
#    orientation_w real,
#    )""")

#c.execute("""INSERT INTO locations VALUES ('Main Reception', 25.365744908596728, 37.27408933229214,
# 0.0, 0.0, 0.4835940951036678, 0.8752923803968962);""")

##################################################
##### FETCHING ALL LOCATIONS FROM DATABASE #######
##################################################

c.execute("""SELECT rowid, name, x_coordinate, y_coordinate, orientation_x, orientation_y, 
orientation_z, orientation_w  FROM locations""")
#c.execute("select * from locations")

""" The list 'locations' below will include all the entries in our database. Each 'location' in the 'locations' 
list will be in the format (rowid, name, x_coordinate, y_coordinate, orientation_x, orientation_y, orientation_z, orientation_w) """

locations = c.fetchall()

conn.commit()
conn.close()

##### SETTING COLUMN INDICES WITH RESPECT TO THEIR POSITION IN THE DATABASE ####

NAME_COLUMN = 1
X_COORDINATE_COLUMN = 2
Y_COORDINATE_COLUMN = 3
ORIENTATION_X_COLUMN = 4
ORIENTATION_Y_COLUMN = 5
ORIENTATION_Z_COLUMN = 6
ORIENTATION_W_COLUMN = 7

current_x_coordinate = 0
current_y_coordinate = 0

######################################
######### UTILITY FUNCTIONS ##########
######################################

def poseCallback(pose_message):
    """Updates the global position variables with the current pose of the robot"""
    global current_x_coordinate
    global current_y_coordinate 
    #global current_quaternion

    current_x_coordinate = pose_message.pose.pose.position.x
    current_y_coordinate = pose_message.pose.pose.position.y


def getLocationInfo(locations, location_index):
    """Returns name, x-coordinate, y-coordinate, and the orientation of the location in locations at the specified """
    
    return locations[location_index][NAME_COLUMN], locations[location_index][X_COORDINATE_COLUMN], locations[location_index][Y_COORDINATE_COLUMN], locations[location_index][ORIENTATION_X_COLUMN], locations[location_index][ORIENTATION_Y_COLUMN], locations[location_index][ORIENTATION_Z_COLUMN], locations[location_index][ORIENTATION_W_COLUMN]


##### CALCULATING EUCLIDEAN DISTANCE TO GOAL #####


def getEuclideanDistance(currentX,currentY, goalX,goalY):
    """Returns a float value euclidian distance. Using math.dist() to calculate the distance. Format is (cX,cY), (gX,gY)."""
    currentLocation = (currentX,currentY)
    goalLocation = (goalX,goalY)
    return math.dist(currentLocation,goalLocation)
    


##### FINDING NEAREST LOCATION FOR TOUR #####

def getNextDestination(locations, tour_locations, current_location):
    """Returns the nearest tour location in locations and removes it from tour_locations.

       tour_locations: list of all the location ids which need to be visited in the tour. (The tour locations selected by the user)
       locations: list of all locations available in our database. A location in locations will be in the format (rowid, name, x_coordinate, y_coordinate,
        orientation_x, orientation_y, orientation_z, orientation_w)
       current_location: a tuple of your current position (x_coordinate, y_coordinate)
    """
    
    """default values for shortest distance and closest location refer to first index in tour_locations"""
    
    current_x_coordinate = current_location[0]
    current_y_coordinate = current_location[1]

    ## Finding nearest location ##
    shortestDistance = getEuclideanDistance(current_x_coordinate,current_y_coordinate
                                            ,getLocationInfo(locations,tour_locations[0])[1],getLocationInfo(locations,tour_locations[0])[2])
    closestLocation = locations[tour_locations[0]]
    closestLocationIndex = tour_locations[0]
    
    ### iterating through the list of distances to find closest tour location to visit ###
    print(len(tour_locations))
    for index in tour_locations:
        nextX = getLocationInfo(locations,index)[1]
        nextY = getLocationInfo(locations,index)[2]
        nextDistance = getEuclideanDistance(current_x_coordinate,current_y_coordinate,nextX,nextY)
        
        if(nextDistance<shortestDistance):
            ## updating the closest tour location ##
            shortestDistance = nextDistance
            closestLocation = index
            closestLocationIndex = index
            
    ## Removing the closest location from tour_locations and returning it ##
    tour_locations.remove(closestLocationIndex)
    return closestLocation

###############################    
######### MOVE TO GOAL ########
###############################

def move_to_goal(xGoal,yGoal, quaternion):

   #define a client for to send goal requests to the move_base server through a SimpleActionClient
   ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

   #wait for the action server to come up
   while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
           rospy.loginfo("Waiting for the move_base action server to come up")

   goal = MoveBaseGoal()
   
   
   #set up the frame parameters
   goal.target_pose.header.frame_id = "map"
   goal.target_pose.header.stamp = rospy.Time.now()

   # moving towards the goal*/

   goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
   goal.target_pose.pose.orientation.x = quaternion[0]
   goal.target_pose.pose.orientation.y = quaternion[1]
   goal.target_pose.pose.orientation.z = quaternion[2]
   goal.target_pose.pose.orientation.w = quaternion[3]

   rospy.loginfo("Sending goal location ...")
   ac.send_goal(goal)

   ac.wait_for_result(rospy.Duration(60))

   if(ac.get_state() ==  GoalStatus.SUCCEEDED):
           rospy.loginfo("You have reached the destination")
           return True

   else:
           rospy.loginfo("The robot failed to reach the destination")
           return False

############################
###### MAIN FUNCTION #######
############################

if __name__ == '__main__':

    #### Initializing a node ####
    rospy.init_node('map_navigation', anonymous=False)

    #### Creating a Subscriber to get current position of robot ####
    position_topic = "amcl_pose"
    pose_subscriber = rospy.Subscriber(position_topic, PoseWithCovarianceStamped, poseCallback) 

    while True:

        #### MAIN INTERFACE ####

        print("\n\nChoose navigation type:")
        print("1. Go to a specific location\n2. Take a tour\n3. Quit")
        print("\nAccepted input: '1', '2', '3':")
        selection1 = input("")


        if(selection1 == '1'):


            ###########################################
            ######  GOING TO SPECIFIC LOCATION  #######
            ###########################################

            ##### INTERFACE #####

            print("\nAvailable locations:")
            for location in locations:
                print("%d. %s" %(location[0], location[1]))
            print("\nChoose location (e.g input '1' if you want to navigate to 'Main Reception'):") #Change if values change

            goal_location_index = int(input()) - 1

            ###### CHECKING IF VALID INPUT WAS ENTERED ######

            while(goal_location_index>=len(locations) or goal_location_index < 0):
                print("\nIncorrect location selected. Please try again: ")
                goal_location_index = int(input()) - 1

            #### GETTING NECESSARY DATA OF THE DESTINATION SELECTED ####

            destination_name, destination_x_coordinate, destination_y_coordinate, orientation_x, orientation_y, orientation_z, orientation_w = getLocationInfo(locations, goal_location_index)
            
            quaternion = [orientation_x, orientation_y, orientation_z, orientation_w]


            print("Going to ", destination_name, "at x:", destination_x_coordinate, "and y:", destination_y_coordinate)

            #### Moving to desired destination ####

            move_to_goal(destination_x_coordinate, destination_y_coordinate, quaternion)

        elif(selection1 == '2'):

            ##############################
            ######  TAKING A TOUR  #######
            ##############################
            
            ##### TOUR SELECTION INTERFACE  #####
            print("\nAvailable locations:")
            for location in locations:
                print("%d. %s" %(location[0], location[1]))
            print("\nChoose location (e.g input '1 3' if you want to navigate to include 'RIOTU Lab' and 'Al-Shegrey Bookstore' in your tour")

            tour_destinations = input().split()

            #### Converting all elements to an int + Converting to a set in order to avoid duplicate inputs ####
            tour_destinations = set(map(int, tour_destinations))

            #### Subtracting 1 to get the corresponding index of location in locations ####
            tour_destinations = list(map(lambda x: x-1, tour_destinations))
            
            ###### CHECKING IF VALID LOCATIONS WERE ENTERED ######

            while(max(tour_destinations) >= len(locations) or min(tour_destinations) < 0):
                print("\nIncorrect location found in given input. Please try again: ")

                tour_destinations = input().split()

                #### Converting all elements to an int + Converting to a set in order to avoid duplicate inputs ####
                tour_destinations = set(map(int, tour_destinations))

                #### Subtracting 1 to get the corresponding index of location in locations ####
                tour_destinations = list(map(lambda x: x-1, tour_destinations))
            
            while(len(tour_destinations) != 0):
                
                #### Getting current position in the map ####
                current_position = [current_x_coordinate, current_y_coordinate]
                
                #### Getting the closest destination in the tour locations relative to current position of the robot ####

                goal_location_index = getNextDestination(locations, tour_destinations, current_position)
                
                #### GETTING NECESSARY DATA OF THE DESTINATION SELECTED ####

                destination_name, destination_x_coordinate, destination_y_coordinate, orientation_x, orientation_y, orientation_z, orientation_w = getLocationInfo(locations, goal_location_index)

                quaternion = [orientation_x, orientation_y, orientation_z, orientation_w]


                print("Going to ", destination_name, "at x:", destination_x_coordinate, "and y:", destination_y_coordinate)
                
                #### Moving to desired destination ####

                move_to_goal(destination_x_coordinate, destination_y_coordinate, quaternion)   
            

        elif(selection1 == '3'):
            print("Quitting..")
            break
        else:
            print("Incorrect input. Try again\n\n")

