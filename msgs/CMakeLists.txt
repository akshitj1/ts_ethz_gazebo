find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set (msgs
  CommandMotorSpeed
  Imu
  MotorSpeed
)
set (msg_files
	CommandMotorSpeed.proto
	Imu.proto
	MotorSpeed.proto
	Float.proto
	Int32.proto
	${PROTOBUF_IMPORT_DIRS}/quaternion.proto
	${PROTOBUF_IMPORT_DIRS}/vector3d.proto
)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msg_files})

add_library(ts_ethz_gazebo_msgs SHARED ${PROTO_SRCS})
target_link_libraries(ts_ethz_gazebo_msgs ${PROTOBUF_LIBRARY})