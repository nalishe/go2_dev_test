


# add_executable(go2_trajectory_follow go2_trajectory_follow.cpp)
# target_link_libraries(go2_trajectory_follow unitree_sdk2)

add_executable(user_vui_client user_vui_client.cpp)
target_link_libraries(user_vui_client unitree_sdk2)

add_executable(user_image_client user_image_client.cpp)
target_link_libraries(user_image_client unitree_sdk2)

# add_executable(user_video_client user_video.client.cpp)
# target_link_libraries(user_video_client unitree_sdk2)