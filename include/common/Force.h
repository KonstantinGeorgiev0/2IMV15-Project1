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
    virtual void apply() = 0;
    virtual void draw() = 0;
};