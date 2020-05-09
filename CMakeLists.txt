cmake_minimum_required(VERSION 3.0.2)
project(gendart)
find_package(catkin REQUIRED COMPONENTS genmsg)

catkin_package(
  CATKIN_DEPENDS genmsg
  CFG_EXTRAS gendart-extras.cmake
)

add_subdirectory(scripts)

file(WRITE ${CATKIN_DEVEL_PREFIX}/${GENMSG_LANGS_DESTINATION}/gendart "DART")
install(FILES ${CATKIN_DEVEL_PREFIX}/${GENMSG_LANGS_DESTINATION}/gendart
  DESTINATION ${GENMSG_LANGS_DESTINATION})

# drop marker file to prevent rospack from spending time on crawling this folder
#file(WRITE ${CATKIN_DEVEL_PREFIX}/share/dart/rospack_nosubdirs "")
#install(FILES ${CATKIN_DEVEL_PREFIX}/share/dart/rospack_nosubdirs
#  DESTINATION share/dart)

catkin_python_setup()
