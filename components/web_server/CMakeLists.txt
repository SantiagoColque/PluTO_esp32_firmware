idf_component_register(SRCS "web_server.c" "handlers_api.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_http_server
                       PRIV_REQUIRES json config_store wifi_manager mqtt_manager
                                     sntp_manager dns_hijack log_ring esp_wifi esp_netif
                                     esp_app_format esp_timer mbedtls lwip)

# The pages are stored uncompressed in git and gzipped into the build
# directory, so the firmware carries roughly a third of the bytes and the
# source stays reviewable.
idf_build_get_property(python PYTHON)
foreach(page index.html logs.html)
    set(page_source "${COMPONENT_DIR}/www/${page}")
    set(page_gzipped "${CMAKE_CURRENT_BINARY_DIR}/${page}.gz")

    add_custom_command(OUTPUT "${page_gzipped}"
                       COMMAND "${python}" "${COMPONENT_DIR}/gzip_asset.py" "${page_source}" "${page_gzipped}"
                       MAIN_DEPENDENCY "${page_source}"
                       DEPENDS "${COMPONENT_DIR}/gzip_asset.py"
                       COMMENT "Compressing ${page}"
                       VERBATIM)

    # BINARY rather than TEXT: the text form appends a NUL byte, which would
    # make the served length one byte longer than the gzip stream.
    target_add_binary_data(${COMPONENT_LIB} "${page_gzipped}" BINARY DEPENDS "${page_gzipped}")
endforeach()
