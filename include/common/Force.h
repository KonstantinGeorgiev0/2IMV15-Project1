//
// Created by Cristiana Carbunaru on 09/05/2026.
//

#ifndef INC_2IMV15_PROJECT1_FORCE_H
#define INC_2IMV15_PROJECT1_FORCE_H

#endif //INC_2IMV15_PROJECT1_FORCE_H

#pragma once

class Force {
public:
    virtual ~Force() {}
    // This calculates and adds the force to the particles.
    virtual void apply() = 0;
    // This draws the force (if applicable, like drawing a spring line).
    virtual void draw() = 0;
};