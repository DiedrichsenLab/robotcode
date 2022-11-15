/*! \mainpage Experimental C++ code of the Motor Control Group 
 *  \author Joern Diedrichsen, Tobias Wiestler, Alexandra Reichenbach, Nick Roach, and Julius Klein 
 *  \date 20/11/2011.
 *  Copyright 2009-2011 Institute of Cognitve Neuoscience. All rights reserved.
 *
 * \section system_sec System requirements 
 * \li Windows XP 
 * \li Microsoft Visual C++ 6.0  
 * \li Sensoray 626 IO card 
 * \li Ideally two monitors (and a dual-head graphics card) for Experimenter and Subject display. 
 *
 * \section intro_sec Introduction
 *
 * This is the documentation for the experimental code for the motor control group at UCL.  
 * The code is written for fast developement of experiments under C++ and with time-sensitive control of robots or other external devices.
 * Time sensitivity is achieved through a hardware interrupt from the Sensoray s626 card. 
 * 
 * \section classes Class Structure
 *  \image html documentation/overview.jpg
 * \subsection classes_abs Abstract classes 
 * The file Experiment.cpp entails the definition of three abstract classes that define much of the default behavior of the experiment. For 
 * example that the program reacts to keyboard input from the user, reads target files that specify how a Block looks like, the running of 
 * a single trial, the recording of data, and the saving of data to disk.
 * 
 * \li Experiment: Contains the main functionality of interacting with the user, setting subject name, and running blocks
 * \li Block:	   Is a list of trials, usually read in from a target file. 
 * \li Trial:	   The abstract class for a single trial. Your class MyTrial will contain the main behavior for your experiment 
 * 
 * All you have to do is to define the behavior of your experiment by defining the classes MyExperiment, MyBlock, MyTrial, and DataRecord. All 
 * other things happen pretty automatically. Do NOT try to change these files - you should not have to (or talk to me if you have suggestions for improving). 
 * This html-documentation contains the doc for a typical robot experiment - Linepush5. This experiment implements reaching movements under 
 * channel movements, or force field movements to a point target, or to a redundant line target. 
 * 
 * \subsection classes_dev Device Classes 
 *
 * The device level contains classes for all kinds of different things that you would want to control. 
 * There are four classes that will be used by nearly all experiments. 
 * \li S626sManager: A wrapper class that serves as API for communication with the s626 card. Allocates channels to other devices
 * \li Timer:		 Relies on one (or two) counters on the s626 card. Provides highly accurate timing information (better than windows at least!) 
 * \li Screen:		 Class that deals with the screen presented to the participant. Uses OpenGL as a drawing commands. Scale and origin can be set flexibly
 * \li TextDisplay:  Text display presented to the experimenter. This takes commands and from the keyboard and can display current status information
 * 
 * Depending on the experiment, you can then use the classes for optimal devices: 
 * \li ManipulandumDR:    Control for the 2-joint robots in QS17, B05. 
 * \li ManipulandumMR:	  Control for the MRI-compatible robots
 * \li ManipulandumRed2D: Control for the Threedom robot in QS33, lab 1. Use with the simple handle end-device. 
 * \li ManipulandumRed3D: Control for the Threedom robot in QS33, lab 1. Use with the wrist device.
 * \li StimulatorBox:	  fMRI-comaptible finger box with vibratory elements 
 * \li TRCounter:		  TR Counter for MRI Experiments over serial port or s626 IO card  
 * \li ...
 * 
 * \subsection classes_util Utility Classes 
 * \li Vector2D:		  2D-vector class 
 * \li Matrix2D:	      2x2 matrix class 
 * \li DataManager:		  Class that holds N data records (per trial) and does memory allocation 
 * \li KalmanFilter:	  Kalman filter for estimation of position and velocity on different channels
 * 
 * \section directory Directory structure
 * It is important that you respect the following directory structure
 * Root-directory is typically c:/robot 
 * <TABLE>
 * <TR> 
 * <TD> source </TD>
 * <TD> Contains all the common .cpp files. DO NOT MODIFY ANY OF THESE FILES! If you need to make changes, please make a copy to your local project directory.
 * If you think you found a bug, or want to add a new feature, please contact Joern Diedrichsen (j.diedrichsen@ucl.ac.uk) to submit your changes. It is strongly 
 * recommended that instead of modifying the existing classes, you inherit the class functionality and make the changes in the derived class. This way you will 
 * stay up-to-date with the newest features and the debugged code. </TD></TR> 
 * <TR><TD> include </TD>
 * <TD> Contains all the common .h (header) files. DO NOT MODIFY ANY OF THESE FILES! See source for details</TD></TR>
 * <TR><TD> documentation </TD>
 * <TD> Contains this documentation and the Doxygen config file</TD></TR>
 * <TR><TD> calib </TD>
 * <TD> Contains calibration and configuration files for Manipulanda, s626card, etc. The parameters are read during the .init() function of each device.
 *  If you need to modify, please make a copy and reference that file.</TD></TR>
 * <TR><TD> project </TD>
 * <TD> Folder for individual projects: Make your own subfolder with your unique project name. Your project folder will contain the following files and folders:
 * \li <MyProject>.cpp:	Contains the source file for your experiment 
 * \li <MyProject>.h:    Contains the header file for your experiment 
 * \li data:				This is where *.mov and *.dat files are written
 * \li target:				This is where the target files are kept
 * </TD></TR>
 *</TABLE> 
 * 
 * \section new_project Making a new Project
 * It's easiest if you start from an exisiting project that is pretty close to what you want to do. 
 * Instead of copying the project, however, you should take the following steps
 * \li New->New Project 
 * \li Choose win32 application (not console application)
 * \li give your project a name and create an empty project
 * \li copy the example project files (.cpp + .h) and rename them to <MyProject>.cpp
 * \li include all the .cpp listed in the Figure under source folder in the project
 * \li Go to c++ project settings in the example project and copy the settings to yours 
 * \li Go to link project settings in the example project and copy the settings to yours
 * \li Compile and start modifying! 
 */
 
