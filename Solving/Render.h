//
// Created by Marco on 28/12/17.
//

#ifndef BUBBLEREGIONSFLOORPLANNER_RENDER_H
#define BUBBLEREGIONSFLOORPLANNER_RENDER_H


#include <GLFW/glfw3.h>
#include "../BaseUtils/Vector2.h"

/** Render the state of the regions and the board
 */
class Render {
    static constexpr int ZOOM_LVL = 16;

public:
    static Render& getINSTANCE(){
        static Render render;
        return render;
    }

private:
    GLFWwindow* window;

    float windowRatio;

    void drawNumber(unsigned char number, Vector2 startPos);
public:

    Vector2 worldPointToScreen(Vector2 wp);

    GLFWwindow *getWindow() const;

    /** Called to initialize the rendering
     */
    void onStart();

    /** Update the screen with the current state
     */
    void onUpdateScreeen();

};


#endif //BUBBLEREGIONSFLOORPLANNER_RENDER_H
