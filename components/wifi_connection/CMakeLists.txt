
# set(DIR_SRCS 
#         "wifi_comm.c")

# idf_component_register(SRCS ${DIR_SRCS} INCLUDE_DIRS ".")
# register_component(
    
idf_component_register(
    SRCS
        "wifi_comm.c" 
    INCLUDE_DIRS
        "."
    REQUIRES 
        
    PRIV_REQUIRES 
        freertos
        )