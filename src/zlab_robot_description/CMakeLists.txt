cmake_minimum_required(VERSION 3.8)
project(zlab_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)

# 安装资源文件
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf/)
install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes/ OPTIONAL)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/)

# 安装配置文件（处理符号链接）
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/ 
        OPTIONAL
        FOLLOW_SYMLINKS)

# 测试配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()