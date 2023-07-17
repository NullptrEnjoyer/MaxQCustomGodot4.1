# CustomGodot4.1.0
 The custom version of godot required to work on MaxQ.
 First, follow build instructions: https://docs.godotengine.org/en/stable/contributing/development/compiling/index.html (Make sure you have 4.1), and compile for visual studio.

 You might need to use an older version of SCONS since at the time of writing the latest doesn't work (try first). In the arguments, add precision=double so we can use larger worlds
 without noticable precision loss, as seen on https://docs.godotengine.org/en/stable/tutorials/physics/large_world_coordinates.html

 Essentially, you navigate to your project via cmd and type scons p=windows vsproj=yes precision=double.

 When opening in visual studio, switch to x64.
 After building, drag the modules into the built engine, and make sure you enable them in the modules folder txt thing. They should be ready for use after you compile


 If you want to export your project you will also need to generate export templates, I found a guide somewhere but idk where it is now lol.
 Take a look at the template bat file, you're basically compiling the engine except it's actually compiling the templates.

 You'll have to do this whenever you added a new class and used it in your project, otherwise the game will crash as it can't find the class.

 So that's great and I have the templates, but where do I put them?

 Great question, friend! In the editor you should go to Project -> Export and read the errors at the bottom of the popup. Make SURE your templates are named correctly, they don't
 do that automatically, and just put them wherever it tells you to. If there are no errors, you're fucked. You can try %appdata%/godot/export_templates if you didn't setup
 self-contained mode (created a file just named "_sc_" in the same directory where your engine .exe is).

 If you have sc mode, it means you already have templates where it expected them and should know where to put them. If not, consult the interwebs.


 Also included:
	-Fallback copy of the current godot 4.0 docs (4.7.2023.)
	-Some bat files