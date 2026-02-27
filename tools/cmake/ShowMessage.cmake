function(showMessage)
  message("------------------- build info ----------------------------------")
  message("-- Build type:" ${CMAKE_BUILD_TYPE})
  message("-- C Standard: " ${CMAKE_C_STANDARD})
  message("-- C++ Standard: " ${CMAKE_CXX_STANDARD})
  message("-- c compiler: " ${CMAKE_C_COMPILER_VERSION})
  message("-- c++ compiler: " ${CMAKE_CXX_COMPILER_VERSION})
  message("-- robot version: " ${GIT_VERSION})
  message("-----------------------------------------------------------------")
  message(
    "    ____  _             ____      ____            __\n"
    "   / __ \\(_)___  __  __/  _/___  / __/___ _____  / /________  __\n"
    "  / /_/ / / __ \\/ / / // // __ \\/ /_/ __ `/ __ \\/ __/ ___/ / / /\n"
    " / ____/ / / / / /_/ // // / / / __/ /_/ / / / / /_/ /  / /_/ /\n"
    "/_/   /_/_/ /_/\\__, /___/_/ /_/_/  \\__,_/_/ /_/\\__/_/   \\__, /\n"
    "              /____/                                   /____/\n")
endfunction()
