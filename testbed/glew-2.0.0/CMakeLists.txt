#----------------------------------------------------------------------------------------------
# Sample code created by Zhang Long. For any problem concerning the code, please contact me:
#   zl2657@arcsoft.com
#   hnex.zhang (Skype)
#-------------------------------------------------------------------------------------------------

cmake_minimum_required(VERSION 2.8.12)

add_library(glew STATIC glew.c)
target_include_directories(glew PUBLIC include)
set_property(TARGET glew PROPERTY FOLDER "libs")
