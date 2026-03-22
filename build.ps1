cd "C:\Users\lucge\physics_sim_project\rocket_sim"
g++ main_sim.cpp math.cpp -o main_sim.exe -I C:\glfw-3.4.bin.WIN64\include -I C:\glew-2.3.1\include -I C:\glad\include -I C:\Users\lucge\physics_sim_project -L C:\glfw-3.4.bin.WIN64\lib-mingw-w64 -L C:\glew-2.3.1\lib\Release\x64 -lglfw3dll -lglew32 -lopengl32

if ($LASTEXITCODE -eq 0) {
    cd "C:\Users\lucge\physics_sim_project\rocket_sim"
    .\main_sim.exe
}

cd "C:\Users\lucge\physics_sim_project"
g++ opengl_stuff/openglwin.cpp rocket_sim/math.cpp C:\glad\src\glad.c -o opengl_stuff/openglwin.exe -I C:\glfw-3.4.bin.WIN64\include -I C:\glew-2.3.1\include -I C:\glad\include -I C:\Users\lucge\physics_sim_project -L C:\glfw-3.4.bin.WIN64\lib-mingw-w64 -L C:\glew-2.3.1\lib\Release\x64 -lglfw3dll -lglew32 -lopengl32

if ($LASTEXITCODE -eq 0) {
    cd "C:\Users\lucge\physics_sim_project"
    .\opengl_stuff\openglwin.exe
}