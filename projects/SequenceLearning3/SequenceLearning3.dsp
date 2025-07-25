# Microsoft Developer Studio Project File - Name="SequenceLearning3" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Application" 0x0101

CFG=SequenceLearning3 - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "SequenceLearning3.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "SequenceLearning3.mak" CFG="SequenceLearning3 - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "SequenceLearning3 - Win32 Release" (based on "Win32 (x86) Application")
!MESSAGE "SequenceLearning3 - Win32 Debug" (based on "Win32 (x86) Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "SequenceLearning3 - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /D "_MBCS" /YX /FD /c
# ADD CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /D "_MBCS" /YX /FD /c
# ADD BASE MTL /nologo /D "NDEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "NDEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x809 /d "NDEBUG"
# ADD RSC /l 0x809 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:windows /machine:I386
# ADD LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:windows /machine:I386

!ELSEIF  "$(CFG)" == "SequenceLearning3 - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /D "_MBCS" /YX /FD /GZ /c
# ADD CPP /nologo /MTd /W3 /Gm /GX /ZI /Od /I "../../include" /I "C:\Program Files\GnuWin32\include" /I "../../DAQ" /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /D "_MBCS" /FR /YX /FD /GZ /c
# ADD BASE MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x809 /d "_DEBUG"
# ADD RSC /l 0x809 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:windows /debug /machine:I386 /pdbtype:sept
# ADD LINK32 opengl32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:windows /pdb:"Debug/freereach1.pdb" /debug /machine:I386 /pdbtype:sept
# SUBTRACT LINK32 /pdb:none

!ENDIF 

# Begin Target

# Name "SequenceLearning3 - Win32 Release"
# Name "SequenceLearning3 - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"
# Begin Source File

SOURCE=..\..\source\Experiment.cpp
# End Source File
# Begin Source File

SOURCE=..\..\source\ManipulandumMR.cpp
# End Source File
# Begin Source File

SOURCE=..\..\source\S626sManager.cpp
# End Source File
# Begin Source File

SOURCE=..\..\source\ScreenMR.cpp
# End Source File
# Begin Source File

SOURCE=.\SequenceLearning3.cpp
# End Source File
# Begin Source File

SOURCE=..\..\source\Serial.cpp
# End Source File
# Begin Source File

SOURCE=..\..\source\StimulatorBox.cpp
# End Source File
# Begin Source File

SOURCE=..\..\source\TextDisplay.cpp
# End Source File
# Begin Source File

SOURCE=..\..\source\Timer626.cpp
# End Source File
# Begin Source File

SOURCE=..\..\source\TRCounter626.cpp
# End Source File
# Begin Source File

SOURCE=..\..\source\WIN626.C
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl"
# Begin Source File

SOURCE=..\..\include\DataManager.h
# End Source File
# Begin Source File

SOURCE=..\..\include\Experiment.h
# End Source File
# Begin Source File

SOURCE=..\..\include\KalmanFilter.h
# End Source File
# Begin Source File

SOURCE=..\..\include\ManipulandumMR.h
# End Source File
# Begin Source File

SOURCE=..\..\include\Matrix2d.h
# End Source File
# Begin Source File

SOURCE=..\..\include\S626sManager.h
# End Source File
# Begin Source File

SOURCE=..\..\include\Screen.h
# End Source File
# Begin Source File

SOURCE=..\..\include\ScreenMR.h
# End Source File
# Begin Source File

SOURCE=.\SequenceLearning3.h
# End Source File
# Begin Source File

SOURCE=..\..\include\Serial.h
# End Source File
# Begin Source File

SOURCE=..\..\include\StimulatorBox.h
# End Source File
# Begin Source File

SOURCE=..\..\include\Target.h
# End Source File
# Begin Source File

SOURCE=..\..\include\TextDisplay.h
# End Source File
# Begin Source File

SOURCE=..\..\include\Timer626.h
# End Source File
# Begin Source File

SOURCE=..\..\include\TRCounter626.h
# End Source File
# Begin Source File

SOURCE=..\..\include\Vector2d.h
# End Source File
# Begin Source File

SOURCE=..\..\include\Win626.h
# End Source File
# End Group
# Begin Group "Resource Files"

# PROP Default_Filter "ico;cur;bmp;dlg;rc2;rct;bin;rgs;gif;jpg;jpeg;jpe"
# End Group
# End Target
# End Project
