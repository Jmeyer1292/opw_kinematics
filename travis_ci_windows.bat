@echo on

call "C:\Program Files (x86)\Microsoft Visual Studio\2017\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
call c:\opt\ros\melodic\x64\setup.bat
catkin_make_isolated
