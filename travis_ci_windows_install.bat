@echo on

call "C:\Program Files (x86)\Microsoft Visual Studio\2017\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
call c:\opt\ros\melodic\x64\setup.bat

rosdep install --from-paths src --ignore-src -r -y

c:\opt\vcpkg\vcpkg install jsoncpp:x64-windows
