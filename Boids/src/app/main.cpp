#include "application.h"
#include <imgui.h>

#include <iostream>
#include <math.h>
#include <deque>
#include <chrono>

#include "../boids/boids.h"


class TestApp : public Application
{
#define COLOR_TEAM_1    nvgRGBA(220,50,50,255)
#define COLOR_TEAM_2    nvgRGBA(50,50,220,255)
#define COLOR_SOLVED    nvgRGBA(50,220,50,255)
#define COLOR_OBSTACLE  nvgRGBA(0, 255, 255, 255)
#define COLOR_LEADER    nvgRGBA(120, 0, 255, 255)


public:

    TestApp(int w, int h, const char * title) : Application(title, w, h) {
        
        ImGui::StyleColorsClassic();
        
        const char* name = IMGUI_FONT_FOLDER"/Cousine-Regular.ttf";
        nvgCreateFont(vg, "sans", name);
        
    }

    void process() override {
        std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
        if(std::chrono::duration_cast<std::chrono::microseconds>(now-lastFrame).count() >= 10./60. * 1.e6)
        {
            if(keyDown[GLFW_KEY_R]) {
                boids.initializePositions(currentMethod);
                initial_radius = boids.getBoid0Radius();
            }
            if(keyDown[GLFW_KEY_SPACE])
                boids.pause();
            if(keyDown[GLFW_KEY_ESCAPE])
                exit(0);
            lastFrame = now;
        }

        if(pressed) {
            auto shift_screen_to_01 = [&](T x, T y, T scale, T width, T height) {
            return TV((x - offset - 0.5 * (0.5 - scale) * width)/(scale * width),
                      (y - offset - 0.5 * (0.5 - scale) * height)/(scale * height));
            };
            double mouse_x = mouseState.lastMouseX;
            double mouse_y = mouseState.lastMouseY;
            mouse_screen = TV(mouse_x, mouse_y);
            TV mouse_01 = shift_screen_to_01(mouse_x, mouse_y, scale, width, height);
            boids.setLeaderPosition(mouse_01);
        }
    }

    void drawImGui() override {

        using namespace ImGui;

       const char* names[] = {"FreeFall", "Separation", "Alignment", "Cohesion", "Leading", "Rotation", "Collision Avoidance", "Teams", "Collab. & Adv."};
       const char* integrationNames[] = {"Symplectic Euler", "Explicit Euler", "Explicit Midpoint"};
       Begin("Menu");
       Combo("Boids Behavior", (int*)&currentMethod, names, 9);
       Combo("Integration method", (int*)&integration, integrationNames, 3);
       //InputDouble("Cohesion force", &(boids.cohesion_strength));
       End();
    }

    double initial_radius;
    void drawNanoVG() override {

        if(currentMethod != previousMethod || previousIntegration != integration) {
            boids.initializePositions(currentMethod);
            previousMethod = currentMethod;
            previousIntegration = integration;
            initial_radius = boids.getBoid0Radius();
        }
        
        boids.updateBehavior(currentMethod, integration);
        
        std::list<TV> boids_pos = boids.getPositions();
        std::list<bool> boids_teams = boids.getTeams();
        
        auto shift_01_to_screen = [&](TV pos_01, T scale, T width, T height) {
            return TV(offset + 0.5 * (0.5 - scale) * width + scale * pos_01[0] * width, offset + 0.5 * (0.5 - scale) * height + scale * pos_01[1] * height);
        };

        auto it_teams = boids_teams.begin();
        for(auto it = boids_pos.begin(); it != boids_pos.end(); it++, it_teams++) {
            TV pos = *it;
            
            // just map position from 01 simulation space to scree space
            // feel free to make changes
            // the only thing that matters is you have pos computed correctly from your simulation
            
            TV screen_pos = shift_01_to_screen(TV(pos[0], pos[1]), scale, width, height);
            nvgBeginPath(vg);
            nvgCircle(vg, screen_pos[0], screen_pos[1], 2.f);
            nvgFillColor(vg, *it_teams? COLOR_TEAM_1 : COLOR_TEAM_2);
            nvgFill(vg);
        }

        TV obstacle_pos = boids.getObstaclePosition();
        TV obstacle_screen_pos = shift_01_to_screen(obstacle_pos, scale, width, height);
        TV zero_vector = TV::Zero();
        TV radius_vector = TV(0, boids.getObstacleRadius());
        TV zero_screen_vector = shift_01_to_screen(zero_vector, scale, width, height);
        TV radius_screen_vector = shift_01_to_screen(radius_vector, scale, width, height);
        double radius_screen = (radius_screen_vector - zero_screen_vector).norm();
        
        if(currentMethod == LEADER || currentMethod == COLLISION_AVOIDANCE) {
            nvgBeginPath(vg);
            nvgCircle(vg, obstacle_screen_pos[0], obstacle_screen_pos[1], radius_screen);
            nvgFillColor(vg, COLOR_OBSTACLE);
            nvgFill(vg);
        }
        
        if(mouse_screen != TV(0, 0) && currentMethod == LEADER) {
            nvgBeginPath(vg);
            nvgCircle(vg, mouse_screen[0], mouse_screen[1], 5);
            nvgFillColor(vg, COLOR_LEADER);
            nvgFill(vg);
        }

        if(currentMethod == ROTATION) {
            nvgBeginPath(vg);
            TV boid_0_vector = TV(0, initial_radius);
            TV boid_0_screen_vector = shift_01_to_screen(boid_0_vector, scale, width, height);
            double boid_0_radius_screen = (boid_0_screen_vector - zero_screen_vector).norm();
            nvgCircle(vg, zero_screen_vector[0], zero_screen_vector[1], boid_0_radius_screen);
            nvgStrokeColor(vg, COLOR_LEADER);
            nvgStroke(vg);

            TV boid0_screen_pos = shift_01_to_screen(TV(boids_pos.front()[0], boids_pos.front()[1]), scale, width, height);
            nvgBeginPath(vg);
            nvgCircle(vg, boid0_screen_pos[0], boid0_screen_pos[1], 4.f);
            nvgFillColor(vg, COLOR_LEADER);
            nvgFill(vg);
        }
    }

protected:
    double offset = 300;
    T scale = 0.3;
    TV mouse_screen = TV(0, 0);
    bool pressed = false;

    void mouseButtonPressed(int button, int mods) override {
        pressed = true;
    }

    void mouseButtonReleased(int button, int mods) override {
        pressed = false;
    }

private:
    int loadFonts(NVGcontext* vg)
    {
        int font;
        font = nvgCreateFont(vg, "sans", "../example/Roboto-Regular.ttf");
        if (font == -1) {
            printf("Could not add font regular.\n");
            return -1;
        }
        font = nvgCreateFont(vg, "sans-bold", "../example/Roboto-Bold.ttf");
        if (font == -1) {
            printf("Could not add font bold.\n");
            return -1;
        }
        return 0;
    }

private:

    MethodTypes previousMethod = COLLAB_ADV;
    MethodTypes currentMethod = previousMethod;
    IntegrationMethods previousIntegration = SYMPLECTIC_EULER;
    IntegrationMethods integration = previousIntegration;


    Boids boids = Boids(40, currentMethod);
    std::chrono::high_resolution_clock::time_point lastFrame;
};

int main(int, char**)
{
    int width = 720;
    int height = 720;
    std::cout << " main " << std::endl;
    TestApp app(width, height, "Assignment 3 Boids");
    app.run();

    return 0;
}
