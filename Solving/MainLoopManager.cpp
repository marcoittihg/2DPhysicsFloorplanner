//
// Created by Marco on 30/12/17.
//

#include <thread>
#include <iostream>
#include "MainLoopManager.h"
#include "Render.h"
#include "Phyisics/Physics.h"

void MainLoopManager::startLoop() {

    Render::getINSTANCE().onStart();

    Physics::getINSTANCE().onStart();

    int phyStepCont= 0;
    while(true /*!glfwWindowShouldClose(Render::getINSTANCE().getWindow())*/) {
        Physics::getINSTANCE().doStep();

        phyStepCont++;

        if(phyStepCont % 10 == 0) {
            Render::getINSTANCE().onUpdateScreeen();
            phyStepCont = 0;
        }
    }

    glfwTerminate();
}
