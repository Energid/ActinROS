# ActinROS
ROS plugins and applications for use with a ROS enabled Energid Actin SDK.

## WINDOWS

1. Download and install the installer.  NOTE: It is recommended to put the installer someplace other than Program Files (i.e. C:/Energid/Installers/...)
2. Windows XP only: download junction.exe.  This is needed to create a link to the {third_party_source} repository from the installer location.  You can download it here: <i>http://technet.microsoft.com/en-us/sysinternals/bb896768</i>
3. Create a link from the ROOT installer directory (i.e. C:/Energid/Installers/Actin_4.1.0.20161028) to the ActinROS directory. This can be done as follows from the command prompt 
  1. cd C:/Energid/Installers/Actin_4.1.0.20161028
  2. Windows XP: junction ActinROS C:/Your/Development/Repositories/ActinROS 
  3. Windows 7/8: mklink /D ActinROS C:/Your/Development/Repositories/ActinROS<br /><br />Alternatively you can use the hard link shell extension utility to accomplish the same procedure from within File Explorer: <i>http://schinagl.priv.at/nt/hardlinkshellext/linkshellextension.html</i>
4. Launch CMake 
5. Set "Where is the source" to the ROOT installer directory  
6. Set "Where to build the binaries" to ROOT/build   (see below)
7. Configure/Generate through Cmake.
  1. During configuration add ActinROS to EC_REPOSITORIES (stable;data;actin_core;toolkits;ActinROS). 
8. Go to ROOT/build and launch Energid.sln. 
9. In Visual Studio build the appropriate projects.

Executables will be built into ROOT/build/bin.  Before running an executable make sure to add the following to the Path environment variable under User variables: %EC_TOOLKITS%\..\bin

## Unix
1. Download and install the installer.
2. Create a link from the ROOT installer directory (i.e. ~/Energid/Actin_4.1.0.20161028) to the ActinROS directory. This can be done as follows from the command prompt 
  1. cd ~/Energid/Actin_4.1.0.20161028 
  2. ln -s ~/Your/Development/Repositories/ActinROS ActinROS
3. Launch CMake 
  1. Set "Where is the source" to the ROOT installer directory  
  2. Set "Where to build the binaries" to ROOT/build   (see below)
4. Configure/Generate through Cmake.
  1. During configuration add ActinROS to EC_REPOSITORIES (stable;data;actin_core;toolkits;ActinROS). 
5. Go to ROOT/build and build using configured make program (make, ninja, etc.) after sourcing SETUP.sh.
