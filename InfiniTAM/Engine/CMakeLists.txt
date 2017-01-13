include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})

add_library(Engine
ImageSourceEngine.cpp
ImageSourceEngine.h
IMUSourceEngine.cpp
IMUSourceEngine.h
Kinect2Engine.cpp
Kinect2Engine.h
OpenNIEngine.cpp
OpenNIEngine.h
LibUVCEngine.cpp
LibUVCEngine.h
UIEngine.cpp
UIEngine.h
CLIEngine.cpp
CLIEngine.h
RealSenseEngine.cpp
RealSenseEngine.h
        DepthSenseEngine.cpp
        DepthSenseEngine.h
)

target_link_libraries(Engine ${GLUT_LIBRARIES})
target_link_libraries(Engine ${OPENGL_LIBRARIES})
target_link_libraries(Engine ITMLib)
IF(WITH_CUDA)
  target_link_libraries(Engine ${CUDA_LIBRARY})
ENDIF()
IF(OPENNI_FOUND)
  target_link_libraries(Engine ${OpenNI_LIBRARY})
ENDIF(OPENNI_FOUND)
IF(DEFINED libuvc_LIBRARIES)
  target_link_libraries(Engine ${libuvc_LIBRARIES})
ENDIF()
IF(WITH_REALSENSE)
  TARGET_LINK_LIBRARIES(Engine ${RealSense_LIBRARY})
  SET_PROPERTY(SOURCE RealSenseEngine.cpp PROPERTY COMPILE_FLAGS -std=c++11)
ENDIF()
IF(WITH_DSSDK)
  TARGET_LINK_LIBRARIES(Engine ${DSSDK_LIBRARIES})
  SET_PROPERTY(SOURCE DepthSenseEngine.cpp PROPERTY COMPILE_FLAGS -std=c++11)
ENDIF()