add_executable(anno_test src/test.cpp)
target_link_libraries(anno_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS anno_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
