find_package(Protobuf REQUIRED)

#function ()
#    add_custom_command(
#        OUTPUT "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}"
#        COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
#            --cpp_out="${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}"
#        DEPENDS ${PROTOBUF_PROTOC_EXECUTABLE} ${proto_files}
#        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
#    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
#    )
#endfunction ()

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
    if(ITR MATCHES ".*gazebo-[0-9.]+$")
        set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
    endif()
endforeach()

set(PROTO_MESSAGES
    set_pusher_request.proto
    add_pushee_request.proto
    ${PROTOBUF_IMPORT_DIRS}/vector2d.proto
    ${PROTOBUF_IMPORT_DIRS}/header.proto
    ${PROTOBUF_IMPORT_DIRS}/time.proto
)

# Generate the protobuf headers.
protobuf_generate_cpp(PROTO_SOURCE PROTO_HEADERS ${PROTO_MESSAGES})
set_source_files_properties(${PROTO_SOURCE} ${PROTO_HEADERS} GENERATED)

# Copy the protobuf headers into the devel space.
foreach (src_path in ${PROTO_HEADERS})
    get_filename_component(header_extension "${src_path}" EXT)
    get_filename_component(header_name "${src_path}" NAME)

    if ("${header_extension}" STREQUAL ".pb.h")
        set(dst_path "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${header_name}")
        list(APPEND PROTOBUF_DEVEL_HEADERS ${dst_path})

        add_custom_command(OUTPUT "${dst_path}"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different "${src_path}" "${dst_path}"
            MAIN_DEPENDENCY "${src_path}"
        )
    endif ()
endforeach ()

add_custom_target("${PROJECT_NAME}_protobuf_headers"
    DEPENDS ${PROTOBUF_DEVEL_HEADERS}
)

# Build a library for the protobuf messages.
add_library("${PROJECT_NAME}_msgs" SHARED ${PROTO_SOURCE})
target_link_libraries("${PROJECT_NAME}_msgs" ${PROTOBUF_LIBRARY})
add_dependencies("${PROJECT_NAME}_msgs" "${PROJECT_NAME}_protobuf_headers")

