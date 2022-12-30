# Only for IntelliJ, do NOT use CMake to build.
# Use Maekfile.js instead.
# More information in NEST.md file.

cmake_minimum_required(VERSION 3.24)
project(CS15466_Game2)

set(CMAKE_CXX_STANDARD 17)

set(NEST_LIBS ../nest-libs/windows)

include_directories(.
        ${NEST_LIBS}/SDL2/include
        ${NEST_LIBS}/glm/include
        ${NEST_LIBS}/libpng/include)

add_executable(CS15466_Game2
        ColorProgram.cpp
        ColorTextureProgram.cpp
        data_path.cpp
        DrawLines.cpp
        GL.cpp
        gl_compile_program.cpp
        LitColorTextureProgram.cpp
        Load.cpp
        load_save_png.cpp
        main.cpp
        Mesh.cpp
        Mode.cpp
        PathFont-font.cpp
        PathFont.cpp
        PlayMode.cpp
        Scene.cpp
        show-meshes.cpp
        show-scene.cpp
        ShowMeshesMode.cpp
        ShowMeshesProgram.cpp
        ShowSceneMode.cpp
        ShowSceneProgram.cpp)
