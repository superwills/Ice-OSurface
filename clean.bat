erase *.sdf
erase *.opensdf

rd .\ipch /s /q

REM remove the build folders (has compiled .exe files in it)
rd .\Debug /s /q
rd .\Release /s /q

rd .\Perlin3D\Debug /s /q
rd .\Perlin3D\Release /s /q

REM Remove the x64 build folder
rd .\x64 /s /q
rd .\Perlin3D\x64 /s /q

