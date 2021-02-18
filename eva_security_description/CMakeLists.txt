cmake_minimum_required(VERSION 2.8.3)
project(eva_security_description)


find_package(catkin REQUIRED 

	urdf
)

catkin_package()


include_directories(

  ${catkin_INCLUDE_DIRS}
)

foreach(dir meshes urdf)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
