##
## Get the Git hash for this code
##
execute_process(
  COMMAND git rev-parse --abbrev-ref HEAD
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE GIT_BRANCH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

execute_process(
  COMMAND git rev-parse --short HEAD
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE GIT_COMMIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

add_definitions("-DGIT_COMMIT_HASH=\"${GIT_COMMIT_HASH}\"")
add_definitions("-DGIT_BRANCH=\"${GIT_BRANCH}\"")


###############################################
# define shared object files
###############################################

add_library(DSA_loop_functions MODULE DSA_loop_functions.h
                                      DSA_loop_functions.cpp
                                      DSA_qt_user_functions.h
                                      DSA_qt_user_functions.cpp)

add_library(DSA_controller     SHARED DSA_controller.h
                                      DSA_controller.cpp
									  DSA_loop_functions.h
                                      DSA_loop_functions.cpp)

###############################################
# link shared object files to dependencies
###############################################

target_link_libraries(DSA_controller
					  BaseController)

target_link_libraries(DSA_loop_functions
                      DSA_controller
                      argos3core_simulator
                      argos3plugin_simulator_entities
                      argos3plugin_simulator_footbot
                      argos3plugin_simulator_genericrobot
                      argos3plugin_simulator_qtopengl
                      ${QT_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
