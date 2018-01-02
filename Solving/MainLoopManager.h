//
// Created by Marco on 30/12/17.
//

#ifndef BUBBLEREGIONSFLOORPLANNER_MAINLOOPMANAGER_H
#define BUBBLEREGIONSFLOORPLANNER_MAINLOOPMANAGER_H


class MainLoopManager {

public:
    static MainLoopManager& getINSTANCE(){
        static MainLoopManager mainLoopManager;
        return mainLoopManager;
    }

    void startLoop();
};


#endif //BUBBLEREGIONSFLOORPLANNER_MAINLOOPMANAGER_H
