cmake_minimum_required(VERSION 3.5)

project(gazebo_model)
#===在进行该代码阅读之前，需要明白文件中的具体路径
#===${CMAKE_CURRENT_SOURCE_DIR} 当前的工程所在src 目录
#===${CMAKE_CURRENT_BINARY_DIR} 运行colcon build 会在 CMakeList.txt 所在目录下生成build/@PROJECT_NAME@  @PROJECT_NAME@为工程名称
#============================================================================
#====Hooks This code snippet is a CMake command used for configuring files, typically used to generate files specific to the build system.  
#===将当前cmakeLists.txt目录下的hooks/hook.dsv.in文件复制到${CMAKE_CURRENT_BINARY_DIR}/hooks/hook.dsv 目录下（如果没有该路径，运行conlcon build 后会创建），并替换hooks/hook.dsv.in文件#===中需要替换的值
#===这里的${CMAKE_CURRENT_BINARY_DIR} 是指运行 colcon build 后产生的build/@PROJECT_NAME@ 路径 @PROJECT_NAME@ 是指工程名称
# SET(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/)
configure_file(
  "hooks/hook.dsv.in"
  "${CMAKE_CURRENT_BINARY_DIR}/hooks/hook.dsv" @ONLY
)
configure_file(
  "hooks/hook.sh.in"
  "${CMAKE_CURRENT_BINARY_DIR}/hooks/hook.sh" @ONLY
)
#============================================================================
#======= Model Generation这段代码的作用是在构建过程中生成一个 SDF 模型文件。具体来说，它调用了一个 Python 脚本 description_generator.py，
#======= 这个脚本用于模板文件生成一个 SDF 模型文件，路径为 
#======= ${CMAKE_CURRENT_SOURCE_DIR}/models/tethys/model.sdf.in。生成的文件放置在构建目录下的 ${CMAKE_CURRENT_BINARY_DIR}/models/tethys/model.sdf 中

add_custom_command(
  OUTPUT modelsdf
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/description_generator.py
  ${CMAKE_CURRENT_SOURCE_DIR}/models/tethys/model.sdf.in
  ${CMAKE_CURRENT_BINARY_DIR}/models/tethys/model.sdf
)
#===this code sets up a custom target in your build system that will execute custom commands or actions defined elsewhere
#===it depends on modelsdf
add_custom_target(generate_model ALL
  DEPENDS modelsdf
)

#===install 命令会将文件安装到share/${PROJECT_NAME}
install(DIRECTORY
  models
  worlds
  ${CMAKE_CURRENT_BINARY_DIR}/hooks
  DESTINATION share/${PROJECT_NAME}
  )
  
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/models/tethys/model.sdf
  DESTINATION share/${PROJECT_NAME}/models/tethys
)  
