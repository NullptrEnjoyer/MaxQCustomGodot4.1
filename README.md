# CustomGodot4.1.0
 The custom version of godot required to work on MaxQ.
 First, follow build instructions: https://docs.godotengine.org/en/stable/contributing/development/compiling/index.html (Make sure you have 4.1), and compile for visual studio.

 You might need to use an older version of SCONS since at the time of writing the latest doesn't work (try first). In the arguments, add precision=double so we can use larger worlds
 without noticable precision loss, as seen on https://docs.godotengine.org/en/stable/tutorials/physics/large_world_coordinates.html

 Essentially, you navigate to your project via cmd and type scons p=windows vsproj=yes precision=double.

 When opening in visual studio, switch to x64.
 After building, drag the modules into the built engine, and make sure you enable them in the modules folder txt thing. They should be ready for use after you compile

 Also included: fallback copy of the current godot 4.0 docs (4.7.2023.)