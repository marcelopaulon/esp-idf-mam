set(srcs "main.c"
        "board.c")

idf_component_register(SRCS "sdOp.c" "${srcs}"
                    INCLUDE_DIRS  ".")
