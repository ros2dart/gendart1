@[if DEVELSPACE]@
# bin variable in develspace
set(GENDART_BIN "@(CMAKE_CURRENT_SOURCE_DIR)/scripts/gen_dart.py")
set(JS_FILES_DIR "@(CMAKE_CURRENT_SOURCE_DIR)/src/gendart")
@[else]@
# bin variable in installspace
set(GENDART_BIN "${gendart_DIR}/../../../@(CATKIN_PACKAGE_BIN_DESTINATION)/gen_dart.py")
set(JS_FILES_DIR "${gendart_DIR}/../../../@(CATKIN_PACKAGE_SHARE_DESTINATION)")
@[end if]@

# Generate .msg or .srv -> .js
# The generated .js files should be added ALL_GEN_OUTPUT_FILES_js
macro(_generate_dart ARG_PKG ARG_MSG ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
  file(MAKE_DIRECTORY ${ARG_GEN_OUTPUT_DIR})

  #Create input and output filenames
  get_filename_component(MSG_NAME ${ARG_MSG} NAME)
  get_filename_component(MSG_SHORT_NAME ${ARG_MSG} NAME_WE)

  set(MSG_GENERATED_NAME ${MSG_SHORT_NAME}.js)
  set(GEN_OUTPUT_FILE ${ARG_GEN_OUTPUT_DIR}/${MSG_GENERATED_NAME})

  assert(CATKIN_ENV)
  add_custom_command(OUTPUT ${GEN_OUTPUT_FILE}
    DEPENDS ${GENDART_BIN} ${ARG_MSG} ${ARG_MSG_DEPS}
    COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENDART_BIN} ${ARG_MSG}
    ${ARG_IFLAGS}
    -p ${ARG_PKG}
    -o ${ARG_GEN_OUTPUT_DIR}
    COMMENT "Generating Javascript code from ${ARG_PKG}/${MSG_NAME}"
    )

  list(APPEND ALL_GEN_OUTPUT_FILES_dart ${GEN_OUTPUT_FILE})

endmacro()

#gendart uses the same program to generate srv and msg files, so call the same macro
macro(_generate_msg_dart ARG_PKG ARG_MSG ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
  _generate_dart(${ARG_PKG} ${ARG_MSG} "${ARG_IFLAGS}" "${ARG_MSG_DEPS}" "${ARG_GEN_OUTPUT_DIR}/msg")
endmacro()

#gendart uses the same program to generate srv and msg files, so call the same macro
macro(_generate_srv_dart ARG_PKG ARG_SRV ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
  _generate_dart(${ARG_PKG} ${ARG_SRV} "${ARG_IFLAGS}" "${ARG_MSG_DEPS}" "${ARG_GEN_OUTPUT_DIR}/srv")
endmacro()

macro(_generate_module_dart ARG_PKG ARG_GEN_OUTPUT_DIR ARG_GENERATED_FILES)
endmacro()

set(dart_INSTALL_DIR share/gendart)
set(gendart_INSTALL_DIR ${dart_INSTALL_DIR}/ros)
