# components/opendroneid/CMakeLists.txt
idf_component_register(
    SRCS "opendroneid.c"
    INCLUDE_DIRS "."
    REQUIRES driver
)