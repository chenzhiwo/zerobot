find_package(PythonInterp REQUIRED)

set(GENMSG_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/genmsg.py PARENT_SCOPE)

function(genmsg MESSAGE_HEADER)
  add_custom_command(
        COMMAND
             ${PYTHON_EXECUTABLE} ${GENMSG_SCRIPT} ${MESSAGE_HEADER} ${ARGN}
        OUTPUT
            ${MESSAGE_HEADER}
        BYPRODUCTS
            ${MESSAGE_HEADER}
        DEPENDS
            ${GENMSG_SCRIPT}
            ${ARGN}
        WORKING_DIRECTORY
            ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT
            "Generating Message Header"
  )
endfunction(genmsg)
