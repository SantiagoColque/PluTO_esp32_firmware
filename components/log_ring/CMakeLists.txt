idf_component_register(SRCS "log_ring_core.c" "log_ring.c"
                       INCLUDE_DIRS "include")

# PUBLIC so every component that includes the header agrees on the size of
# log_ring_t.
target_compile_definitions(${COMPONENT_LIB} PUBLIC LOG_RING_CAPACITY=${CONFIG_PLUTO_LOG_RING_LINES})
