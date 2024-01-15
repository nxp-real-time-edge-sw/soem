if(NOT COMPONENT_SOEM_INCLUDED)

    set(COMPONENT_SOEM_INCLUDED true CACHE BOOL "component_soem component is included.")

    target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/soem/ethercatbase.c
        ${CMAKE_CURRENT_LIST_DIR}/soem/ethercatcoe.c
        ${CMAKE_CURRENT_LIST_DIR}/soem/ethercatconfig.c
        ${CMAKE_CURRENT_LIST_DIR}/soem/ethercatdc.c
        ${CMAKE_CURRENT_LIST_DIR}/soem/ethercateoe.c
        ${CMAKE_CURRENT_LIST_DIR}/soem/ethercatfoe.c
        ${CMAKE_CURRENT_LIST_DIR}/soem/ethercatmain.c
        ${CMAKE_CURRENT_LIST_DIR}/soem/ethercatprint.c
        ${CMAKE_CURRENT_LIST_DIR}/soem/ethercatsoe.c
        ${CMAKE_CURRENT_LIST_DIR}/oshw/mcux-sdk/nicdrv.c
        ${CMAKE_CURRENT_LIST_DIR}/oshw/mcux-sdk/oshw.c
        ${CMAKE_CURRENT_LIST_DIR}/oshw/mcux-sdk/soem_port.c
		${CMAKE_CURRENT_LIST_DIR}/osal/freertos/osal.c
		${CMAKE_CURRENT_LIST_DIR}/oshw/mcux-sdk/enet/enet.c
    )
    target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/soem/
        ${CMAKE_CURRENT_LIST_DIR}/oshw/mcux-sdk/
        ${CMAKE_CURRENT_LIST_DIR}/osal/
		${CMAKE_CURRENT_LIST_DIR}/osal/freertos/
		${CMAKE_CURRENT_LIST_DIR}/oshw/mcux-sdk/enet/
    )

endif()
