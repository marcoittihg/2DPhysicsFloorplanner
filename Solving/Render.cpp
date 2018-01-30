//
// Created by Marco on 28/12/17.
//

#include <iostream>
#include "Render.h"
#include "../Data/FPGAData/Board.h"
#include "Phyisics/Physics.h"

void Render::onStart() {

    /* Initialize the library */
    if (!glfwInit())
        return;

    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(1500, 700, "Floorplanner", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return;
    }
    glfwSwapInterval(1);

    /* Make the window's context current */
    glfwMakeContextCurrent(window);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

Vector2 Render::worldPointToScreen(Vector2 wp) {
    wp.setX(wp.getX() / (float)ZOOM_LVL);
    wp.setY(wp.getY() / (float)ZOOM_LVL * windowRatio);

    return wp;
}

void Render::onUpdateScreeen() {
    glClear(GL_COLOR_BUFFER_BIT);

    //Draw board
    Board* board = Physics::getINSTANCE().getBoard();

    glBegin(GL_LINE_LOOP);

    int windowWidth, windowHeight;
    glfwGetWindowSize(window, &windowWidth, &windowHeight);

    windowRatio = (float)windowWidth / (float)windowHeight;

    Vector2 boardHalfDimension = Vector2(board->getDimension().get_x(), board->getDimension().get_y());
    boardHalfDimension.multiply(0.5);

    Vector2 minuHalfBoardDim = boardHalfDimension;
    minuHalfBoardDim.multiply(-1);

    Vector2 boardEdges = worldPointToScreen(boardHalfDimension);

    glVertex3f(-boardEdges.getX(),+boardEdges.getY(),0);
    glVertex3f(+boardEdges.getX(),+boardEdges.getY(),0);
    glVertex3f(+boardEdges.getX(),-boardEdges.getY(),0);
    glVertex3f(-boardEdges.getX(),-boardEdges.getY(),0);

    glEnd();

    float yStep = 2 * boardEdges.getY() / board->getDimension().get_y();
    float xStep = 2 * boardEdges.getX() / board->getDimension().get_x();

    glColor4f(1,1,1,0.1);

    for (int i = 0; i < board->getDimension().get_x(); ++i) {
        glBegin(GL_LINE_LOOP);
        glVertex3f(-boardEdges.getX() + xStep * i,+boardEdges.getY(),0);
        glVertex3f(-boardEdges.getX() + xStep * i,-boardEdges.getY(),0);
        glEnd();
    }
    for (int i = 0; i < board->getDimension().get_y(); ++i) {
        glBegin(GL_LINE_LOOP);
        glVertex3f(+boardEdges.getX(),boardEdges.getY() - yStep * i,0);
        glVertex3f(-boardEdges.getX(),boardEdges.getY() - yStep * i,0);
        glEnd();
    }

    glColor4f(1,1,1,1);

    //Draw regions
    PhysicsRegion* regions = Physics::getINSTANCE().getPhysicsRegions();
    unsigned short regionsNum = Physics::getINSTANCE().getRegionNum();
    for (int i = 0; i < regionsNum; ++i) {
        PhysicsRegion region = regions[i];
        Rigidbody* rb = region.getRb();

        if(region.getRegionState() == PhysicsRegionState::FLOATING){
            glColor4f(1,1,1,1);
        }else{
            glColor4f(0,1,0,1);
        }

        Vector2 regionDim = rb->getDimension();
        regionDim.multiply(0.5);

        Vector2 regionTopRight = rb->getPosition();
        regionTopRight.add(regionDim);
        regionTopRight = worldPointToScreen(regionTopRight);

        Vector2 regionBottomLeft = rb->getPosition();
        regionDim.multiply(-1);
        regionBottomLeft.add(regionDim);
        regionBottomLeft = worldPointToScreen(regionBottomLeft);



        glBegin(GL_LINE_LOOP);

        glVertex3f(regionBottomLeft.getX(),regionTopRight.getY(),0);
        glVertex3f(regionTopRight.getX(),regionTopRight.getY(),0);
        glVertex3f(regionTopRight.getX(),regionBottomLeft.getY(),0);
        glVertex3f(regionBottomLeft.getX(),regionBottomLeft.getY(),0);

        glEnd();

        drawNumber(region.getRegionIndex(), Vector2(regionBottomLeft.getX() - 0.5 / ZOOM_LVL, regionTopRight.getY() - 0.6 / ZOOM_LVL));

        //Draw anchor points

        Vector2 regionPos = rb->getPosition();
        regionPos = worldPointToScreen(regionPos);

        Vector2 prefAnchorPos = region.getPreferedAnchorPoint();
        prefAnchorPos =worldPointToScreen(prefAnchorPos);


        glColor4f(1,0,0,0.5);
        glBegin(GL_LINE_STRIP);
        glVertex3f(regionPos.getX(), regionPos.getY(),0);
        glVertex3f(prefAnchorPos.getX(), prefAnchorPos.getY(), 0);
        glEnd();

        //Draw IOs connections
        for (int j = 0; j < region.getIONum(); ++j) {
            RegionIOData ioData = region.getRegionIO()[j];
            Vector2 ioPos = Vector2(ioData.getPortColumn(), ioData.getPortRow());

            ioPos.add(minuHalfBoardDim);
            ioPos.add(Vector2(-0.5,-0.5));

            ioPos = worldPointToScreen(ioPos);

            glColor4f(1,1,1,0.5);
            glBegin(GL_LINE_STRIP);
            glVertex3f(regionPos.getX(), regionPos.getY(),0);
            glVertex3f(ioPos.getX(), ioPos.getY(), 0);
            glEnd();

        }

        //Draw interconnections
        for (int j = 0; j < region.getInterconnectedRegions().size(); ++j) {
            PhysicsRegion* intercRegion = region.getInterconnectedRegions().at(j);

            Vector2 intercRegionPos = intercRegion->getRb()->getPosition();
            intercRegionPos = worldPointToScreen(intercRegionPos);


            glColor4f(1,1,1,0.3);
            glBegin(GL_LINE_STRIP);
            glVertex3f(regionPos.getX(), regionPos.getY(),0);
            glVertex3f(intercRegionPos.getX(), intercRegionPos.getY(), 0);
            glEnd();
        }
    }

    glColor4f(1,1,1,1);

    glfwSwapBuffers(window);

    glfwPollEvents();

}

GLFWwindow *Render::getWindow() const {
    return window;
}

void Render::drawNumber(unsigned char number, Vector2 startPos) {
    int numDigit = 1;

    int n = number / 10;
    while (n != 0){
        n /= 10;
        numDigit++;
    }

    for (int i = 0; i < numDigit; ++i) {
        glBegin(GL_LINE_STRIP);

        int num = ((int)number) % 10;
        float padd = (float) (numDigit - i) / ZOOM_LVL;

        switch (num) {
            case 0:
                glVertex3f(startPos.getX() + 0.2 / ZOOM_LVL + padd, startPos.getY() + 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() - 0.2 / ZOOM_LVL+ padd, startPos.getY() + 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() - 0.2 / ZOOM_LVL+ padd, startPos.getY() - 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() + 0.2 / ZOOM_LVL+ padd, startPos.getY() - 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() + 0.2 / ZOOM_LVL + padd, startPos.getY() + 0.5 / ZOOM_LVL, 0);
                break;

            case 1:
                glVertex3f(startPos.getX() + padd - 0.2 / ZOOM_LVL, startPos.getY() + 0.2 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() + padd , startPos.getY() + 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() + padd , startPos.getY() - 0.5 / ZOOM_LVL, 0);
                break;

            case 2:
                glVertex3f(startPos.getX() - 0.25 / ZOOM_LVL + padd, startPos.getY() + 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() + 0.25 / ZOOM_LVL+ padd, startPos.getY() + 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() + 0.25 / ZOOM_LVL+ padd, startPos.getY(), 0);
                glVertex3f(startPos.getX() - 0.25 / ZOOM_LVL+ padd, startPos.getY(), 0);
                glVertex3f(startPos.getX() - 0.25 / ZOOM_LVL+ padd, startPos.getY() - 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() + 0.25 / ZOOM_LVL+ padd, startPos.getY() - 0.5 / ZOOM_LVL, 0);
                break;

            case 3:
                glVertex3f(startPos.getX() - 0.25 / ZOOM_LVL + padd, startPos.getY() + 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() + 0.25 / ZOOM_LVL+ padd, startPos.getY() + 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() + 0.25 / ZOOM_LVL+ padd, startPos.getY(), 0);
                glVertex3f(startPos.getX() - 0.25 / ZOOM_LVL+ padd, startPos.getY(), 0);
                glVertex3f(startPos.getX() + 0.25 / ZOOM_LVL+ padd, startPos.getY(), 0);
                glVertex3f(startPos.getX() + 0.25 / ZOOM_LVL+ padd, startPos.getY() - 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() - 0.25 / ZOOM_LVL+ padd, startPos.getY() - 0.5 / ZOOM_LVL, 0);
                break;

            case 4:
                glVertex3f(startPos.getX() + padd , startPos.getY() - 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() + padd , startPos.getY() + 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() - 0.25 / ZOOM_LVL+ padd, startPos.getY(), 0);
                glVertex3f(startPos.getX() + 0.20 / ZOOM_LVL+ padd, startPos.getY(), 0);
                break;

            case 5:
                glVertex3f(startPos.getX() + 0.25 / ZOOM_LVL + padd, startPos.getY() + 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() - 0.25 / ZOOM_LVL+ padd, startPos.getY() + 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() - 0.25 / ZOOM_LVL+ padd, startPos.getY(), 0);
                glVertex3f(startPos.getX() + 0.25 / ZOOM_LVL+ padd, startPos.getY(), 0);
                glVertex3f(startPos.getX() + 0.25 / ZOOM_LVL+ padd, startPos.getY() - 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() - 0.25 / ZOOM_LVL+ padd, startPos.getY() - 0.5 / ZOOM_LVL, 0);
                break;

            case 6:
                glVertex3f(startPos.getX() + 0.2 / ZOOM_LVL + padd, startPos.getY() + 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() - 0.2 / ZOOM_LVL+ padd, startPos.getY() + 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() - 0.2 / ZOOM_LVL+ padd, startPos.getY(), 0);
                glVertex3f(startPos.getX() + 0.2 / ZOOM_LVL+ padd, startPos.getY(), 0);
                glVertex3f(startPos.getX() + 0.2 / ZOOM_LVL+ padd, startPos.getY() - 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() - 0.2 / ZOOM_LVL+ padd, startPos.getY() - 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() - 0.2 / ZOOM_LVL+ padd, startPos.getY(), 0);

                break;

            case 7:
                glVertex3f(startPos.getX() - 0.25 / ZOOM_LVL + padd, startPos.getY() + 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() + 0.25 / ZOOM_LVL+ padd, startPos.getY() + 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() - 0.25 / ZOOM_LVL+ padd, startPos.getY() - 0.5 / ZOOM_LVL, 0);
                break;

            case 8:
                glVertex3f(startPos.getX() + 0.2 / ZOOM_LVL + padd, startPos.getY() + 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() - 0.2 / ZOOM_LVL+ padd, startPos.getY() + 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() - 0.2 / ZOOM_LVL+ padd, startPos.getY(), 0);
                glVertex3f(startPos.getX() + 0.2 / ZOOM_LVL+ padd, startPos.getY(), 0);
                glVertex3f(startPos.getX() + 0.2 / ZOOM_LVL+ padd, startPos.getY() - 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() - 0.2 / ZOOM_LVL+ padd, startPos.getY() - 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() - 0.2 / ZOOM_LVL+ padd, startPos.getY(), 0);
                glVertex3f(startPos.getX() + 0.2 / ZOOM_LVL+ padd, startPos.getY(), 0);
                glVertex3f(startPos.getX() + 0.2 / ZOOM_LVL + padd, startPos.getY() + 0.5 / ZOOM_LVL, 0);
                break;

            case 9:
                glVertex3f(startPos.getX() - 0.2 / ZOOM_LVL + padd, startPos.getY() - 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() + 0.2 / ZOOM_LVL+ padd, startPos.getY() - 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() + 0.2 / ZOOM_LVL+ padd, startPos.getY(), 0);
                glVertex3f(startPos.getX() - 0.2 / ZOOM_LVL+ padd, startPos.getY(), 0);
                glVertex3f(startPos.getX() - 0.2 / ZOOM_LVL+ padd, startPos.getY() + 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() + 0.2 / ZOOM_LVL+ padd, startPos.getY() + 0.5 / ZOOM_LVL, 0);
                glVertex3f(startPos.getX() + 0.2 / ZOOM_LVL+ padd, startPos.getY(), 0);
                break;
        }

        glEnd();

        number /= 10;
    }
}
