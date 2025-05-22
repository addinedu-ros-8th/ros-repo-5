cmake_minimum_required(VERSION 3.5)
project(controll_server_package_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 메시지 및 서비스 파일 설정
set(msg_files
  "msg/TaxiState.msg"
)

set(srv_files
  "srv/CmdMoveTo.srv"
  "srv/TaxiEvent.srv"
)

# 메시지 생성 설정
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_package()
