set(COMPONENT_SRCDIRS " .")
set(COMPONENT_ADD_INCLUDEDIRS ".")

register_component()


target_link_libraries(${COMPONENT_TARGET} "-u bt_passkey")
