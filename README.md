# BulletPerformanceTest

A code snippet to be included into the Bullet repository to compare performance between Jolt and Bullet

To run:

- Clone the Bullet repository at https://github.com/bulletphysics/bullet3.git
- Add this repository as a submodule in examples\PerformanceTest
- Edit examples\CMakeLists.txt and add PerformanceTest to the SUBDIRS command at the top of the file
- Follow the build instructions of the Bullet library to compile the library and all examples, e.g. in the root of the project:
-- md build
-- cd build
-- cmake -DBULLET2_MULTITHREADING=ON -G "Visual Studio 17 2022" ..