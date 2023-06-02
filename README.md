# PathSaver

**This is my contribution to a course project that I have developed.**

PathSaver is an application extension that enhances travel planning and optimization. It introduces several key features to simplify route calculations and improve overall efficiency.

## Save Shortest Paths for Future Use

One of the main features of PathSaver is the ability to save the shortest paths between two locations for future use. This eliminates the need to recalculate paths every time the same route is required. When a user finds the shortest path using Dijkstra's algorithm, the application stores this path for future reference. Users can easily access the stored paths without the time-consuming recalculation process. This feature saves valuable time and resources, making the application more efficient and user-friendly.

## Greedy Approach for Efficient Trip Planning

PathSaver introduces a smart "Greedy Approach" feature that optimizes trip planning. Users can start at one location and efficiently visit every other location in the sequence that minimizes travel time. By avoiding unnecessary backtracking and optimizing the route planning, PathSaver significantly reduces travel time and improves overall efficiency. This feature is particularly useful for users who frequently travel or need to plan complex routes with multiple destinations.

## Consideration of Speed Limits

To provide accurate estimations of travel time, PathSaver takes into account the speed limits for different road types. The application incorporates speed limits into its calculations, ensuring more precise route planning. Users have two options to consider speed limits: relying on a third-party service that provides real-time speed limit updates or using estimations provided by PathSaver. While relying on the third-party service guarantees up-to-date speed limits, using estimations simplifies the process at the cost of some accuracy.

/-------------------------------------------------------------------------
/ Starter Code and GUI Application for Course 3 in the
/ Java Programming: Object Oriented Design of 
/ Data Structures Specialization:
/
/ Advanced Data Structures in Java
/ https://www.coursera.org/learn/advanced-data-structures
/
/ Authored by UCSD MOOC Team:
/ Mia Minnes, Christine Alvarado, Leo Porter, Alec Brickner
/ and Adam Setters
/
/ Date: 12/16/2015
/-------------------------------------------------------------------------

---------------------------------------------------------[ DESCRIPTION ]--

The files provided are skeleton code, as well as grading previews and 
testing files to be used in completing the course programming 
assignments. Additionally, you are provided a runnable JavaFX program 
which will help to test and demonstrate your implementations.

-------------------------------------------------------[ FILES BY WEEK ]--

Below are the files introduced in each week and used in each week
of the course. See file for description...

Week 1 : Introduction to the course and graphs
==============================================
basicgraph.Graph.java
basicgraph.GraphAdjList.java
basicgraph.GraphAdjMatrix.java

Week 2 : Class design and simple graph search
==================================================
roadgraph.MapGraph.java
week2example.Maze.java
week2example.MazeLoader.java
week2example.MazeNode.java

Utility files
=============
geography.GeographicPoint.java
geography.RoadSegment.java
util.GraphLoader.java

---------------------------------------------------------------[ SETUP ]-- 

Importing Project into eclipse:
	1. Create a new Java Project in your workspace
	2. Import the starter files:
	  File -> Import -> Select "File System" -> Next -> Browse and set 
	  root directory to folder contents of zip were extracted to -> Finish

Feel free to use another IDE or manually compile and run your programs.
If you need help, google is your friend.
