package(default_visibility = ["//visibility:public"])

# Configs are grouped per robot cell. "common" holds the files that are
# genuinely cell-independent; every other directory is one cell.
_CONFIG_GLOB = [
    "*/*.pbtxt",
    "*/*.textproto",
    "*/*.yaml",
]

exports_files(glob(_CONFIG_GLOB))

filegroup(
    name = "all_configs",
    srcs = glob(_CONFIG_GLOB),
)

filegroup(
    name = "all_world_updates",
    srcs = glob(["*/*.updates.pbtxt"]),
)
