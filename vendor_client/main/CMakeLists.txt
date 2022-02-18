set(srcs "main.c"
        "board.c"
        "hashmap.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS  ".")
