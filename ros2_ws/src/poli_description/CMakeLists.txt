cmake_minimum_required(VERSION 3.8)
project(poli_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  # colcon test: xacro가 변환되고 check_urdf가 통과하는지 확인
  ament_add_test(check_urdf
    COMMAND sh -c "xacro ${CMAKE_CURRENT_SOURCE_DIR}/urdf/poli.urdf.xacro > ${CMAKE_CURRENT_BINARY_DIR}/poli.urdf && check_urdf ${CMAKE_CURRENT_BINARY_DIR}/poli.urdf"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
endif()

ament_package()
