#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>

// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
int main(int argc, char** argv)
{
    BVHAccel::SplitMethod method;
    if (argc >= 2 && std::string(argv[1]) == "SAH") {
        printf(" - Generating SAH...\n\n");
        method = BVHAccel::SplitMethod::SAH;
    } else {
        printf(" - Generating BVH...\n\n");
        method = BVHAccel::SplitMethod::NAIVE;
    }

    Scene scene(1280, 960, method);

    printf("Object Build.\n");
    MeshTriangle bunny("../models/bunny/bunny.obj", method);

    scene.Add(&bunny);
    scene.Add(std::make_unique<Light>(Vector3f(-20, 70, 20), 1));
    scene.Add(std::make_unique<Light>(Vector3f(20, 70, 20), 1));
    printf("Scene Build.\n");
    scene.buildBVH();

    Renderer r;

    auto start = std::chrono::system_clock::now();
    r.Render(scene);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count() << " microseconds\n";

    return 0;
}