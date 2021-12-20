set(COMPONENT_SRCS "app_main.c" "app_wifi.c" "app_camera.c" )
#set(COMPONENT_SRCS "sdcard_camera.c")
set(COMPONENT_ADD_INCLUDEDIRS "include")

set(COMPONENT_REQUIRES
	esp32-camera
    nvs_flash
	fatfs
    )

#set(COMPONENT_REQUIRES
    #esp32-camera
#    nvs_flash
#    )

register_component()
